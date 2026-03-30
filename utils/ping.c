/* Copyright (c) 2026
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* utils/ping.c
 * ICMP echo utility over raw Ethernet devices.
 */

#include "utils.h"
#include <endian.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <lunix/net.h>

#define DEFAULT_DEVICE "/dev/eth0"
#define DEFAULT_COUNT UINT_MAX
#define DEFAULT_INTERVAL_MS 1000
#define DEFAULT_TIMEOUT_MS 1000
#define DEFAULT_PAYLOAD_SIZE 56

#define ETHERTYPE_IPV4 0x0800
#define ETHERTYPE_ARP 0x0806

#define ARP_HARDWARE_ETHERNET 1
#define ARP_OPERATION_REQUEST 1
#define ARP_OPERATION_REPLY 2

#define IPV4_VERSION 4
#define IPV4_MIN_IHL 5
#define IPV4_PROTOCOL_ICMP 1
#define IPV4_PROTOCOL_UDP 17
#define IPV4_TTL 64
#define IPV4_FLAG_DONT_FRAGMENT 0x4000
#define IPV4_FLAG_MORE_FRAGMENTS 0x2000
#define IPV4_FRAGMENT_OFFSET_MASK 0x1FFF

#define ICMP_TYPE_ECHO_REPLY 0
#define ICMP_TYPE_ECHO_REQUEST 8

#define UDP_DNS_PORT 53

#define DNS_FLAG_RESPONSE 0x8000
#define DNS_FLAG_TRUNCATED 0x0200
#define DNS_FLAG_RECURSION_DESIRED 0x0100
#define DNS_RCODE_MASK 0x000F
#define DNS_RECORD_A 1
#define DNS_CLASS_IN 1
#define DNS_MAX_MESSAGE 512
#define DNS_MAX_NAME_WIRE 255
#define NETWORK_CONFIG_DIR "/run/network"
#define NETWORK_CONFIG_PATH_MAX 256

#define MAX_ICMP_PAYLOAD (NET_DEFAULT_MTU - 20 - 8)

struct EthernetHeader {
    uint8_t destination[6];
    uint8_t source[6];
    uint16_t etherType;
} __attribute__((packed));

struct ArpPacket {
    uint16_t hardwareType;
    uint16_t protocolType;
    uint8_t hardwareLength;
    uint8_t protocolLength;
    uint16_t operation;
    uint8_t senderMac[6];
    uint8_t senderIp[4];
    uint8_t targetMac[6];
    uint8_t targetIp[4];
} __attribute__((packed));

struct Ipv4Header {
    uint8_t versionAndIhl;
    uint8_t dscpAndEcn;
    uint16_t totalLength;
    uint16_t identification;
    uint16_t flagsAndFragmentOffset;
    uint8_t ttl;
    uint8_t protocol;
    uint16_t headerChecksum;
    uint8_t sourceIp[4];
    uint8_t destinationIp[4];
} __attribute__((packed));

struct UdpHeader {
    uint16_t sourcePort;
    uint16_t destinationPort;
    uint16_t length;
    uint16_t checksum;
} __attribute__((packed));

struct IcmpEchoHeader {
    uint8_t type;
    uint8_t code;
    uint16_t checksum;
    uint16_t identifier;
    uint16_t sequence;
} __attribute__((packed));

struct DnsHeader {
    uint16_t id;
    uint16_t flags;
    uint16_t questionCount;
    uint16_t answerCount;
    uint16_t authorityCount;
    uint16_t additionalCount;
} __attribute__((packed));

struct StoredNetworkConfig {
    bool haveSource;
    bool haveGateway;
    bool haveNetmask;
    bool haveNameserver;
    uint8_t sourceIp[4];
    uint8_t gatewayIp[4];
    uint8_t netmask[4];
    uint8_t nameserverIp[4];
};

static volatile sig_atomic_t interrupted = 0;

static uint32_t addChecksumWords(uint32_t sum, const void* buffer,
        size_t length);
static uint16_t calculateChecksum(const void* buffer, size_t length);
static uint16_t calculateTransportChecksum(const uint8_t sourceIp[4],
        const uint8_t destinationIp[4], uint8_t protocol, const void* buffer,
        size_t length);
static bool determineNextHop(const uint8_t localIp[4], const uint8_t netmask[4],
        const uint8_t targetIp[4], const uint8_t gatewayIp[4],
        bool haveGateway, uint8_t nextHopIp[4]);
static bool drainMatchingArpReply(int fd, const uint8_t localIp[4],
        const uint8_t targetIp[4], uint8_t targetMac[6]);
static int64_t elapsedNanoseconds(const struct timespec* start,
        const struct timespec* end);
static bool encodeDnsName(const char* hostname, uint8_t* buffer,
        size_t bufferSize, size_t* encodedLength);
static void formatIpv4(const uint8_t address[4], char buffer[16]);
static bool getIpv4Payload(const uint8_t* frame, size_t frameLength,
        const uint8_t expectedSourceIp[4], const uint8_t expectedDestinationIp[4],
        uint8_t expectedProtocol, const uint8_t** payload, size_t* payloadLength,
        uint8_t* ttl);
static bool handleArpReply(const uint8_t* frame, size_t length,
        const uint8_t localIp[4], const uint8_t targetIp[4],
        uint8_t targetMac[6]);
static bool handleDnsResponse(const uint8_t* frame, size_t frameLength,
        const uint8_t localIp[4], const uint8_t serverIp[4],
        uint16_t sourcePort, uint16_t queryId, uint8_t resolvedIp[4]);
static bool handleEchoReply(const uint8_t* frame, size_t frameLength,
        const uint8_t localIp[4], const uint8_t targetIp[4],
        uint16_t identifier, uint16_t sequence, uint8_t* ttl,
        size_t* icmpBytes);
static void handleSignal(int signalNumber);
static bool loadStoredNetworkConfig(const char* device,
        struct StoredNetworkConfig* config);
static bool isQemuUserNetwork(const uint8_t address[4]);
static bool parseIpv4(const char* string, uint8_t address[4]);
static long parseLong(const char* string, const char* option, long min,
        long max);
static void printDurationMilliseconds(int64_t nanoseconds);
static void printStatistics(const char* targetString, unsigned int transmitted,
        unsigned int received, int64_t minRtt, int64_t totalRtt,
        int64_t maxRtt);
static bool resolveHostname(int fd, const uint8_t localMac[6],
        const uint8_t localIp[4], const uint8_t nameserverIp[4],
        const uint8_t nameserverMac[6], unsigned int timeoutMs,
        const char* hostname, uint8_t resolvedIp[4]);
static bool resolveTargetMac(int fd, const uint8_t localMac[6],
        const uint8_t localIp[4], const uint8_t targetIp[4],
        unsigned int timeoutMs, uint8_t targetMac[6]);
static void sendArpRequest(int fd, const uint8_t localMac[6],
        const uint8_t localIp[4], const uint8_t targetIp[4]);
static void sendDnsQuery(int fd, const uint8_t localMac[6],
        const uint8_t serverMac[6], const uint8_t localIp[4],
        const uint8_t serverIp[4], uint16_t sourcePort, uint16_t queryId,
        const char* hostname);
static void sendEchoRequest(int fd, const uint8_t localMac[6],
        const uint8_t nextHopMac[6], const uint8_t localIp[4],
        const uint8_t targetIp[4], uint16_t identifier, uint16_t sequence,
        size_t payloadSize);
static bool sameSubnet(const uint8_t address1[4], const uint8_t address2[4],
        const uint8_t netmask[4]);
static bool skipDnsName(const uint8_t* message, size_t messageLength,
        size_t* offset);
static void sleepMilliseconds(int milliseconds);
static bool waitForDnsReply(int fd, const uint8_t localIp[4],
        const uint8_t serverIp[4], uint16_t sourcePort, uint16_t queryId,
        unsigned int timeoutMs, uint8_t resolvedIp[4]);
static bool waitForEchoReply(int fd, const uint8_t localIp[4],
        const uint8_t targetIp[4], uint16_t identifier, uint16_t sequence,
        unsigned int timeoutMs, uint8_t* ttl, size_t* icmpBytes);

int main(int argc, char* argv[]) {
    static const struct option longopts[] = {
        { "count", required_argument, 0, 'c' },
        { "device", required_argument, 0, 'd' },
        { "gateway", required_argument, 0, 'g' },
        { "interval", required_argument, 0, 'i' },
        { "netmask", required_argument, 0, 'm' },
        { "nameserver", required_argument, 0, 'n' },
        { "size", required_argument, 0, 's' },
        { "source", required_argument, 0, 'S' },
        { "timeout", required_argument, 0, 'W' },
        { "help", no_argument, 0, 0 },
        { "version", no_argument, 0, 1 },
        { 0, 0, 0, 0 }
    };

    const char* device = DEFAULT_DEVICE;
    const char* sourceString = NULL;
    const char* gatewayString = NULL;
    const char* netmaskString = NULL;
    const char* nameserverString = NULL;
    unsigned int count = DEFAULT_COUNT;
    unsigned int intervalMs = DEFAULT_INTERVAL_MS;
    unsigned int timeoutMs = DEFAULT_TIMEOUT_MS;
    size_t payloadSize = DEFAULT_PAYLOAD_SIZE;

    int c;
    while ((c = getopt_long(argc, argv, "c:d:g:i:m:n:s:S:W:", longopts,
            NULL)) != -1) {
        switch (c) {
        case 0:
            return help(argv[0], "[OPTIONS] DESTINATION\n"
                    "  -c, --count=COUNT        stop after COUNT requests\n"
                    "  -d, --device=DEVICE      raw ethernet device "
                    "(default: " DEFAULT_DEVICE ")\n"
                    "  -g, --gateway=ADDRESS    default IPv4 gateway\n"
                    "  -i, --interval=MS        delay between requests in "
                    "milliseconds\n"
                    "  -m, --netmask=ADDRESS    local IPv4 netmask "
                    "(default: 255.255.255.0)\n"
                    "  -n, --nameserver=ADDRESS DNS server for hostnames\n"
                    "  -s, --size=BYTES         ICMP payload size "
                    "(default: 56)\n"
                    "  -S, --source=ADDRESS     local IPv4 address\n"
                    "  -W, --timeout=MS         reply timeout in milliseconds\n"
                    "      --help               display this help\n"
                    "      --version            display version info\n"
                    "\n"
                    "This utility sends ARP, DNS and ICMP frames directly "
                    "through /dev/ethN.\n"
                    "For QEMU user-mode networking, 10.0.2.2 and 10.0.2.3 are "
                    "used automatically\n"
                    "as gateway and nameserver when the source address is in "
                    "10.0.2.0/24.");
        case 1:
            return version(argv[0]);
        case 'c':
            count = (unsigned int) parseLong(optarg, "--count", 1, UINT_MAX);
            break;
        case 'd':
            device = optarg;
            break;
        case 'g':
            gatewayString = optarg;
            break;
        case 'i':
            intervalMs = (unsigned int) parseLong(optarg, "--interval", 0,
                    INT_MAX);
            break;
        case 'm':
            netmaskString = optarg;
            break;
        case 'n':
            nameserverString = optarg;
            break;
        case 's':
            payloadSize = (size_t) parseLong(optarg, "--size", 0,
                    MAX_ICMP_PAYLOAD);
            break;
        case 'S':
            sourceString = optarg;
            break;
        case 'W':
            timeoutMs = (unsigned int) parseLong(optarg, "--timeout", 1,
                    INT_MAX);
            break;
        case '?':
            return 1;
        }
    }

    if (optind + 1 != argc) {
        errx(1, optind >= argc ? "missing destination address" :
                "extra operand: '%s'", argv[optind + 1]);
    }

    struct StoredNetworkConfig storedConfig = {};
    loadStoredNetworkConfig(device, &storedConfig);

    uint8_t localIp[4];
    if (sourceString) {
        if (!parseIpv4(sourceString, localIp)) {
            errx(1, "invalid source address: '%s'", sourceString);
        }
    } else if (storedConfig.haveSource) {
        memcpy(localIp, storedConfig.sourceIp, sizeof(localIp));
    } else {
        errx(1, "missing source address, use -S ADDRESS or run dhcp");
    }

    uint8_t netmask[4] = { 255, 255, 255, 0 };
    if (netmaskString) {
        if (!parseIpv4(netmaskString, netmask)) {
            errx(1, "invalid netmask: '%s'", netmaskString);
        }
    } else if (storedConfig.haveNetmask) {
        memcpy(netmask, storedConfig.netmask, sizeof(netmask));
    }

    bool haveGateway = false;
    uint8_t gatewayIp[4] = { 0, 0, 0, 0 };
    if (gatewayString) {
        if (!parseIpv4(gatewayString, gatewayIp)) {
            errx(1, "invalid gateway address: '%s'", gatewayString);
        }
        haveGateway = true;
    } else if (storedConfig.haveGateway) {
        memcpy(gatewayIp, storedConfig.gatewayIp, sizeof(gatewayIp));
        haveGateway = true;
    }

    bool haveNameserver = false;
    uint8_t nameserverIp[4] = { 0, 0, 0, 0 };
    if (nameserverString) {
        if (!parseIpv4(nameserverString, nameserverIp)) {
            errx(1, "invalid nameserver address: '%s'", nameserverString);
        }
        haveNameserver = true;
    } else if (storedConfig.haveNameserver) {
        memcpy(nameserverIp, storedConfig.nameserverIp, sizeof(nameserverIp));
        haveNameserver = true;
    }

    if (isQemuUserNetwork(localIp)) {
        if (!haveGateway) {
            gatewayIp[0] = 10;
            gatewayIp[1] = 0;
            gatewayIp[2] = 2;
            gatewayIp[3] = 2;
            haveGateway = true;
        }
        if (!haveNameserver) {
            nameserverIp[0] = 10;
            nameserverIp[1] = 0;
            nameserverIp[2] = 2;
            nameserverIp[3] = 3;
            haveNameserver = true;
        }
    }

    int fd = open(device, O_RDWR);
    if (fd < 0) err(1, "open: '%s'", device);

    struct net_info info;
    if (ioctl(fd, NET_GET_INFO, &info) < 0) {
        err(1, "ioctl(NET_GET_INFO): '%s'", device);
    }
    if (!(info.flags & NET_INFO_LINK_UP)) {
        errx(1, "%s: link is down", device);
    }

    if (payloadSize + sizeof(struct Ipv4Header) +
            sizeof(struct IcmpEchoHeader) > info.mtu) {
        errx(1, "payload size %zu exceeds MTU %u", payloadSize, info.mtu);
    }

    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = handleSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);

    const char* destinationArgument = argv[optind];
    uint8_t targetIp[4];
    bool resolvedName = false;
    if (!parseIpv4(destinationArgument, targetIp)) {
        if (!haveNameserver) {
            errx(1, "cannot resolve '%s' without a nameserver, use -n ADDRESS "
                    "or run dhcp", destinationArgument);
        }

        uint8_t dnsNextHopIp[4];
        if (!determineNextHop(localIp, netmask, nameserverIp, gatewayIp,
                haveGateway, dnsNextHopIp)) {
            errx(1, "nameserver %s is outside the local subnet, use "
                    "-g ADDRESS", nameserverString ? nameserverString :
                    "configured nameserver");
        }

        uint8_t nameserverMac[6];
        if (!resolveTargetMac(fd, info.mac, localIp, dnsNextHopIp, timeoutMs,
                nameserverMac)) {
            char nameserverAddress[16];
            formatIpv4(dnsNextHopIp, nameserverAddress);
            errx(1, "no ARP reply from nameserver route %s", nameserverAddress);
        }

        if (!resolveHostname(fd, info.mac, localIp, nameserverIp,
                nameserverMac, timeoutMs, destinationArgument, targetIp)) {
            errx(1, "failed to resolve '%s'", destinationArgument);
        }
        resolvedName = true;
    }

    uint8_t nextHopIp[4];
    if (!determineNextHop(localIp, netmask, targetIp, gatewayIp, haveGateway,
            nextHopIp)) {
        errx(1, "destination %s is outside the local subnet, use -g ADDRESS",
                destinationArgument);
    }

    char sourceAddress[16];
    char targetAddress[16];
    formatIpv4(localIp, sourceAddress);
    formatIpv4(targetIp, targetAddress);

    if (resolvedName) {
        printf("PING %s (%s) from %s via %s: %zu data bytes\n",
                destinationArgument, targetAddress, sourceAddress, device,
                payloadSize);
    } else {
        printf("PING %s from %s via %s: %zu data bytes\n", targetAddress,
                sourceAddress, device, payloadSize);
    }

    uint8_t nextHopMac[6];
    bool haveNextHopMac = false;
    uint16_t identifier = (uint16_t) getpid();
    unsigned int transmitted = 0;
    unsigned int received = 0;
    int64_t minRtt = 0;
    int64_t maxRtt = 0;
    int64_t totalRtt = 0;
    bool hadFailure = false;

    for (unsigned int sequence = 1; sequence <= count && !interrupted;
            sequence++) {
        struct timespec iterationStart;
        clock_gettime(CLOCK_MONOTONIC, &iterationStart);

        if (!haveNextHopMac) {
            haveNextHopMac = resolveTargetMac(fd, info.mac, localIp,
                    nextHopIp, timeoutMs, nextHopMac);
            if (!haveNextHopMac) {
                hadFailure = true;
                warnx("no ARP reply from route to %s", targetAddress);
                break;
            }
        }

        struct timespec sendTime;
        clock_gettime(CLOCK_MONOTONIC, &sendTime);
        sendEchoRequest(fd, info.mac, nextHopMac, localIp, targetIp,
                identifier, (uint16_t) sequence, payloadSize);
        transmitted++;

        uint8_t ttl = 0;
        size_t icmpBytes = 0;
        if (waitForEchoReply(fd, localIp, targetIp, identifier,
                (uint16_t) sequence, timeoutMs, &ttl, &icmpBytes)) {
            struct timespec receiveTime;
            clock_gettime(CLOCK_MONOTONIC, &receiveTime);
            int64_t rtt = elapsedNanoseconds(&sendTime, &receiveTime);
            if (received == 0 || rtt < minRtt) minRtt = rtt;
            if (received == 0 || rtt > maxRtt) maxRtt = rtt;
            totalRtt += rtt;
            received++;

            printf("%zu bytes from %s: icmp_seq=%u ttl=%u time=", icmpBytes,
                    targetAddress, sequence, ttl);
            printDurationMilliseconds(rtt);
            puts(" ms");
        } else if (!interrupted) {
            hadFailure = true;
            printf("Request timeout for icmp_seq %u\n", sequence);
            haveNextHopMac = false;
        }

        if (sequence == count || interrupted) break;

        struct timespec iterationEnd;
        clock_gettime(CLOCK_MONOTONIC, &iterationEnd);
        int64_t elapsed = elapsedNanoseconds(&iterationStart, &iterationEnd);
        int64_t sleepNs = (int64_t) intervalMs * 1000000 - elapsed;
        if (sleepNs > 0) {
            sleepMilliseconds((int) ((sleepNs + 999999) / 1000000));
        }
    }

    printStatistics(resolvedName ? destinationArgument : targetAddress,
            transmitted, received, minRtt, totalRtt, maxRtt);

    close(fd);
    return !hadFailure && received == transmitted ? 0 : 1;
}

static uint32_t addChecksumWords(uint32_t sum, const void* buffer,
        size_t length) {
    const uint8_t* bytes = buffer;

    while (length > 1) {
        sum += ((uint32_t) bytes[0] << 8) | bytes[1];
        bytes += 2;
        length -= 2;
    }
    if (length > 0) {
        sum += (uint32_t) bytes[0] << 8;
    }

    return sum;
}

static uint16_t calculateChecksum(const void* buffer, size_t length) {
    uint32_t sum = addChecksumWords(0, buffer, length);

    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return (uint16_t) ~sum;
}

static uint16_t calculateTransportChecksum(const uint8_t sourceIp[4],
        const uint8_t destinationIp[4], uint8_t protocol, const void* buffer,
        size_t length) {
    uint8_t pseudoHeader[12];
    memcpy(pseudoHeader, sourceIp, 4);
    memcpy(pseudoHeader + 4, destinationIp, 4);
    pseudoHeader[8] = 0;
    pseudoHeader[9] = protocol;
    pseudoHeader[10] = (uint8_t) (length >> 8);
    pseudoHeader[11] = (uint8_t) length;

    uint32_t sum = addChecksumWords(0, pseudoHeader, sizeof(pseudoHeader));
    sum = addChecksumWords(sum, buffer, length);
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    uint16_t result = (uint16_t) ~sum;
    return result ? result : 0xFFFF;
}

static bool determineNextHop(const uint8_t localIp[4], const uint8_t netmask[4],
        const uint8_t targetIp[4], const uint8_t gatewayIp[4],
        bool haveGateway, uint8_t nextHopIp[4]) {
    if (sameSubnet(localIp, targetIp, netmask)) {
        memcpy(nextHopIp, targetIp, 4);
        return true;
    }

    if (!haveGateway) return false;

    memcpy(nextHopIp, gatewayIp, 4);
    return true;
}

static bool drainMatchingArpReply(int fd, const uint8_t localIp[4],
        const uint8_t targetIp[4], uint8_t targetMac[6]) {
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    uint8_t frame[NET_MAX_FRAME_SIZE];

    while (!interrupted) {
        int result = poll(&pfd, 1, 0);
        if (result <= 0) return false;
        if (!(pfd.revents & (POLLIN | POLLRDNORM))) return false;

        ssize_t bytesRead = read(fd, frame, sizeof(frame));
        if (bytesRead < 0) {
            if (errno == EINTR) continue;
            err(1, "read");
        }
        if (handleArpReply(frame, (size_t) bytesRead, localIp, targetIp,
                targetMac)) {
            return true;
        }
    }

    return false;
}

static int64_t elapsedNanoseconds(const struct timespec* start,
        const struct timespec* end) {
    return (end->tv_sec - start->tv_sec) * 1000000000LL +
            (end->tv_nsec - start->tv_nsec);
}

static bool encodeDnsName(const char* hostname, uint8_t* buffer,
        size_t bufferSize, size_t* encodedLength) {
    size_t offset = 0;
    const char* label = hostname;
    const char* current = hostname;

    while (true) {
        if (*current == '.' || *current == '\0') {
            size_t labelLength = current - label;
            if (labelLength == 0 || labelLength > 63) return false;
            if (offset + 1 + labelLength + 1 > bufferSize) return false;

            buffer[offset++] = labelLength;
            memcpy(buffer + offset, label, labelLength);
            offset += labelLength;

            if (*current == '\0') {
                buffer[offset++] = 0;
                *encodedLength = offset;
                return true;
            }

            label = current + 1;
        }
        current++;
    }
}

static void formatIpv4(const uint8_t address[4], char buffer[16]) {
    snprintf(buffer, 16, "%u.%u.%u.%u", address[0], address[1], address[2],
            address[3]);
}

static bool getIpv4Payload(const uint8_t* frame, size_t frameLength,
        const uint8_t expectedSourceIp[4], const uint8_t expectedDestinationIp[4],
        uint8_t expectedProtocol, const uint8_t** payload, size_t* payloadLength,
        uint8_t* ttl) {
    if (frameLength < sizeof(struct EthernetHeader) + sizeof(struct Ipv4Header)) {
        return false;
    }

    const struct EthernetHeader* ethernetHeader =
            (const struct EthernetHeader*) frame;
    if (be16toh(ethernetHeader->etherType) != ETHERTYPE_IPV4) return false;

    const uint8_t* ipBuffer = frame + sizeof(*ethernetHeader);
    const struct Ipv4Header* ipv4 = (const struct Ipv4Header*) ipBuffer;
    uint8_t version = ipv4->versionAndIhl >> 4;
    uint8_t ihl = ipv4->versionAndIhl & 0xF;
    size_t headerLength = (size_t) ihl * 4;

    if (version != IPV4_VERSION || ihl < IPV4_MIN_IHL ||
            frameLength < sizeof(*ethernetHeader) + headerLength) {
        return false;
    }
    if (calculateChecksum(ipBuffer, headerLength) != 0) return false;

    uint16_t totalLength = be16toh(ipv4->totalLength);
    if (totalLength < headerLength ||
            frameLength < sizeof(*ethernetHeader) + totalLength) {
        return false;
    }

    uint16_t fragmentInfo = be16toh(ipv4->flagsAndFragmentOffset);
    if (ipv4->protocol != expectedProtocol ||
            (fragmentInfo & IPV4_FLAG_MORE_FRAGMENTS) != 0 ||
            (fragmentInfo & IPV4_FRAGMENT_OFFSET_MASK) != 0) {
        return false;
    }
    if (expectedSourceIp &&
            memcmp(ipv4->sourceIp, expectedSourceIp, 4) != 0) {
        return false;
    }
    if (expectedDestinationIp &&
            memcmp(ipv4->destinationIp, expectedDestinationIp, 4) != 0) {
        return false;
    }

    *payload = ipBuffer + headerLength;
    *payloadLength = totalLength - headerLength;
    if (ttl) *ttl = ipv4->ttl;
    return true;
}

static bool handleArpReply(const uint8_t* frame, size_t length,
        const uint8_t localIp[4], const uint8_t targetIp[4],
        uint8_t targetMac[6]) {
    if (length < sizeof(struct EthernetHeader) + sizeof(struct ArpPacket)) {
        return false;
    }

    const struct EthernetHeader* ethernetHeader =
            (const struct EthernetHeader*) frame;
    if (be16toh(ethernetHeader->etherType) != ETHERTYPE_ARP) return false;

    const struct ArpPacket* arp =
            (const struct ArpPacket*) (frame + sizeof(*ethernetHeader));
    if (be16toh(arp->hardwareType) != ARP_HARDWARE_ETHERNET ||
            be16toh(arp->protocolType) != ETHERTYPE_IPV4 ||
            arp->hardwareLength != 6 || arp->protocolLength != 4 ||
            be16toh(arp->operation) != ARP_OPERATION_REPLY) {
        return false;
    }
    if (memcmp(arp->senderIp, targetIp, 4) != 0 ||
            memcmp(arp->targetIp, localIp, 4) != 0) {
        return false;
    }

    memcpy(targetMac, arp->senderMac, 6);
    return true;
}

static bool handleDnsResponse(const uint8_t* frame, size_t frameLength,
        const uint8_t localIp[4], const uint8_t serverIp[4],
        uint16_t sourcePort, uint16_t queryId, uint8_t resolvedIp[4]) {
    const uint8_t* payload;
    size_t payloadLength;
    if (!getIpv4Payload(frame, frameLength, serverIp, localIp,
            IPV4_PROTOCOL_UDP, &payload, &payloadLength, NULL)) {
        return false;
    }

    if (payloadLength < sizeof(struct UdpHeader) + sizeof(struct DnsHeader)) {
        return false;
    }

    const struct UdpHeader* udp = (const struct UdpHeader*) payload;
    uint16_t udpLength = be16toh(udp->length);
    if (be16toh(udp->sourcePort) != UDP_DNS_PORT ||
            be16toh(udp->destinationPort) != sourcePort ||
            udpLength < sizeof(*udp) + sizeof(struct DnsHeader) ||
            udpLength > payloadLength) {
        return false;
    }

    const uint8_t* dnsBuffer = payload + sizeof(*udp);
    size_t dnsLength = udpLength - sizeof(*udp);
    const struct DnsHeader* dns = (const struct DnsHeader*) dnsBuffer;
    uint16_t flags = be16toh(dns->flags);
    if (be16toh(dns->id) != queryId ||
            !(flags & DNS_FLAG_RESPONSE) ||
            (flags & DNS_FLAG_TRUNCATED) ||
            (flags & DNS_RCODE_MASK) != 0) {
        return false;
    }

    size_t offset = sizeof(*dns);
    uint16_t questionCount = be16toh(dns->questionCount);
    uint16_t answerCount = be16toh(dns->answerCount);

    for (uint16_t i = 0; i < questionCount; i++) {
        if (!skipDnsName(dnsBuffer, dnsLength, &offset) ||
                offset + 4 > dnsLength) {
            return false;
        }
        offset += 4;
    }

    for (uint16_t i = 0; i < answerCount; i++) {
        if (!skipDnsName(dnsBuffer, dnsLength, &offset) ||
                offset + 10 > dnsLength) {
            return false;
        }

        uint16_t type = be16toh(*(const uint16_t*) (dnsBuffer + offset));
        uint16_t recordClass = be16toh(*(const uint16_t*)
                (dnsBuffer + offset + 2));
        uint16_t dataLength = be16toh(*(const uint16_t*)
                (dnsBuffer + offset + 8));
        offset += 10;

        if (offset + dataLength > dnsLength) {
            return false;
        }

        if (type == DNS_RECORD_A && recordClass == DNS_CLASS_IN &&
                dataLength == 4) {
            memcpy(resolvedIp, dnsBuffer + offset, 4);
            return true;
        }

        offset += dataLength;
    }

    return false;
}

static bool handleEchoReply(const uint8_t* frame, size_t frameLength,
        const uint8_t localIp[4], const uint8_t targetIp[4],
        uint16_t identifier, uint16_t sequence, uint8_t* ttl,
        size_t* icmpBytes) {
    const uint8_t* payload;
    size_t payloadLength;
    if (!getIpv4Payload(frame, frameLength, targetIp, localIp,
            IPV4_PROTOCOL_ICMP, &payload, &payloadLength, ttl)) {
        return false;
    }

    if (payloadLength < sizeof(struct IcmpEchoHeader)) return false;

    const struct IcmpEchoHeader* icmp = (const struct IcmpEchoHeader*) payload;
    if (icmp->type != ICMP_TYPE_ECHO_REPLY || icmp->code != 0 ||
            be16toh(icmp->identifier) != identifier ||
            be16toh(icmp->sequence) != sequence) {
        return false;
    }
    if (calculateChecksum(payload, payloadLength) != 0) return false;

    *icmpBytes = payloadLength;
    return true;
}

static void handleSignal(int signalNumber) {
    (void) signalNumber;
    interrupted = 1;
}

static bool loadStoredNetworkConfig(const char* device,
        struct StoredNetworkConfig* config) {
    memset(config, 0, sizeof(*config));

    const char* interfaceName = strrchr(device, '/');
    interfaceName = interfaceName ? interfaceName + 1 : device;
    if (!*interfaceName) return false;

    char path[NETWORK_CONFIG_PATH_MAX];
    if (snprintf(path, sizeof(path), "%s/%s.conf", NETWORK_CONFIG_DIR,
            interfaceName) >= (int) sizeof(path)) {
        return false;
    }

    FILE* file = fopen(path, "r");
    if (!file) return false;

    char* line = NULL;
    size_t lineSize = 0;
    while (getline(&line, &lineSize, file) > 0) {
        char* value = strchr(line, '=');
        if (!value) continue;

        *value++ = '\0';
        char* newline = strchr(value, '\n');
        if (newline) *newline = '\0';

        uint8_t address[4];
        if (!parseIpv4(value, address)) continue;

        if (strcmp(line, "ip") == 0) {
            memcpy(config->sourceIp, address, sizeof(address));
            config->haveSource = true;
        } else if (strcmp(line, "gateway") == 0) {
            memcpy(config->gatewayIp, address, sizeof(address));
            config->haveGateway = true;
        } else if (strcmp(line, "netmask") == 0) {
            memcpy(config->netmask, address, sizeof(address));
            config->haveNetmask = true;
        } else if (strcmp(line, "nameserver") == 0) {
            memcpy(config->nameserverIp, address, sizeof(address));
            config->haveNameserver = true;
        }
    }

    free(line);
    fclose(file);
    return config->haveSource;
}

static bool isQemuUserNetwork(const uint8_t address[4]) {
    return address[0] == 10 && address[1] == 0 && address[2] == 2;
}

static bool parseIpv4(const char* string, uint8_t address[4]) {
    const char* current = string;

    for (int i = 0; i < 4; i++) {
        char* end;
        errno = 0;
        unsigned long value = strtoul(current, &end, 10);
        if (errno != 0 || end == current || value > 255) {
            return false;
        }

        address[i] = (uint8_t) value;
        if (i == 3) {
            return *end == '\0';
        }
        if (*end != '.') return false;
        current = end + 1;
    }

    return false;
}

static long parseLong(const char* string, const char* option, long min,
        long max) {
    char* end;
    errno = 0;
    long value = strtol(string, &end, 10);
    if (errno != 0 || end == string || *end != '\0' ||
            value < min || value > max) {
        errx(1, "invalid value for %s: '%s'", option, string);
    }
    return value;
}

static void printDurationMilliseconds(int64_t nanoseconds) {
    int64_t microseconds = nanoseconds / 1000;
    printf("%jd.%03jd", (intmax_t) (microseconds / 1000),
            (intmax_t) (microseconds % 1000));
}

static void printStatistics(const char* targetString, unsigned int transmitted,
        unsigned int received, int64_t minRtt, int64_t totalRtt,
        int64_t maxRtt) {
    printf("\n--- %s ping statistics ---\n", targetString);
    unsigned int lost = transmitted - received;
    unsigned int loss = transmitted == 0 ? 0 : lost * 100 / transmitted;
    printf("%u packets transmitted, %u packets received, %u%% packet loss\n",
            transmitted, received, loss);

    if (received > 0) {
        int64_t avgRtt = totalRtt / received;
        printf("round-trip min/avg/max = ");
        printDurationMilliseconds(minRtt);
        putchar('/');
        printDurationMilliseconds(avgRtt);
        putchar('/');
        printDurationMilliseconds(maxRtt);
        puts(" ms");
    }
}

static bool resolveHostname(int fd, const uint8_t localMac[6],
        const uint8_t localIp[4], const uint8_t nameserverIp[4],
        const uint8_t nameserverMac[6], unsigned int timeoutMs,
        const char* hostname, uint8_t resolvedIp[4]) {
    uint16_t queryId = (uint16_t) (getpid() ^ time(NULL));
    uint16_t sourcePort = (uint16_t) (49152 + ((queryId ^ getpid()) & 0x3FFF));

    for (int attempt = 0; attempt < 3 && !interrupted; attempt++) {
        sendDnsQuery(fd, localMac, nameserverMac, localIp, nameserverIp,
                sourcePort, queryId, hostname);
        if (waitForDnsReply(fd, localIp, nameserverIp, sourcePort, queryId,
                timeoutMs, resolvedIp)) {
            return true;
        }
        queryId++;
        sourcePort++;
    }

    return false;
}

static bool resolveTargetMac(int fd, const uint8_t localMac[6],
        const uint8_t localIp[4], const uint8_t targetIp[4],
        unsigned int timeoutMs, uint8_t targetMac[6]) {
    if (drainMatchingArpReply(fd, localIp, targetIp, targetMac)) return true;

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    uint8_t frame[NET_MAX_FRAME_SIZE];

    for (int attempt = 0; attempt < 3 && !interrupted; attempt++) {
        sendArpRequest(fd, localMac, localIp, targetIp);

        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);

        while (!interrupted) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            int64_t elapsed = elapsedNanoseconds(&start, &now);
            int64_t remainingNs = (int64_t) timeoutMs * 1000000 - elapsed;
            if (remainingNs <= 0) break;

            int timeout = (int) ((remainingNs + 999999) / 1000000);
            int result = poll(&pfd, 1, timeout);
            if (result < 0) {
                if (errno == EINTR) continue;
                err(1, "poll");
            }
            if (result == 0) break;
            if (!(pfd.revents & (POLLIN | POLLRDNORM))) continue;

            ssize_t bytesRead = read(fd, frame, sizeof(frame));
            if (bytesRead < 0) {
                if (errno == EINTR) continue;
                err(1, "read");
            }

            if (handleArpReply(frame, (size_t) bytesRead, localIp, targetIp,
                    targetMac)) {
                return true;
            }
        }
    }

    return false;
}

static void sendArpRequest(int fd, const uint8_t localMac[6],
        const uint8_t localIp[4], const uint8_t targetIp[4]) {
    uint8_t frame[sizeof(struct EthernetHeader) + sizeof(struct ArpPacket)];
    struct EthernetHeader* ethernetHeader = (struct EthernetHeader*) frame;
    struct ArpPacket* arp = (struct ArpPacket*) (frame +
            sizeof(*ethernetHeader));

    memset(ethernetHeader->destination, 0xFF,
            sizeof(ethernetHeader->destination));
    memcpy(ethernetHeader->source, localMac, sizeof(ethernetHeader->source));
    ethernetHeader->etherType = htobe16(ETHERTYPE_ARP);

    arp->hardwareType = htobe16(ARP_HARDWARE_ETHERNET);
    arp->protocolType = htobe16(ETHERTYPE_IPV4);
    arp->hardwareLength = 6;
    arp->protocolLength = 4;
    arp->operation = htobe16(ARP_OPERATION_REQUEST);
    memcpy(arp->senderMac, localMac, sizeof(arp->senderMac));
    memcpy(arp->senderIp, localIp, sizeof(arp->senderIp));
    memset(arp->targetMac, 0, sizeof(arp->targetMac));
    memcpy(arp->targetIp, targetIp, sizeof(arp->targetIp));

    if (write(fd, frame, sizeof(frame)) < 0) {
        err(1, "write");
    }
}

static void sendDnsQuery(int fd, const uint8_t localMac[6],
        const uint8_t serverMac[6], const uint8_t localIp[4],
        const uint8_t serverIp[4], uint16_t sourcePort, uint16_t queryId,
        const char* hostname) {
    uint8_t dnsPayload[DNS_MAX_MESSAGE];
    struct DnsHeader* dns = (struct DnsHeader*) dnsPayload;
    dns->id = htobe16(queryId);
    dns->flags = htobe16(DNS_FLAG_RECURSION_DESIRED);
    dns->questionCount = htobe16(1);
    dns->answerCount = 0;
    dns->authorityCount = 0;
    dns->additionalCount = 0;

    size_t dnsLength = sizeof(*dns);
    size_t encodedNameLength;
    if (!encodeDnsName(hostname, dnsPayload + dnsLength,
            sizeof(dnsPayload) - dnsLength - 4, &encodedNameLength)) {
        errx(1, "unsupported host name: '%s'", hostname);
    }
    dnsLength += encodedNameLength;

    *(uint16_t*) (dnsPayload + dnsLength) = htobe16(DNS_RECORD_A);
    dnsLength += 2;
    *(uint16_t*) (dnsPayload + dnsLength) = htobe16(DNS_CLASS_IN);
    dnsLength += 2;

    uint8_t frame[NET_MAX_FRAME_SIZE];
    const size_t frameLength = sizeof(struct EthernetHeader) +
            sizeof(struct Ipv4Header) + sizeof(struct UdpHeader) + dnsLength;

    struct EthernetHeader* ethernetHeader = (struct EthernetHeader*) frame;
    struct Ipv4Header* ipv4 = (struct Ipv4Header*) (frame +
            sizeof(*ethernetHeader));
    struct UdpHeader* udp = (struct UdpHeader*) ((uint8_t*) ipv4 +
            sizeof(*ipv4));
    uint8_t* payload = (uint8_t*) (udp + 1);

    memcpy(ethernetHeader->destination, serverMac,
            sizeof(ethernetHeader->destination));
    memcpy(ethernetHeader->source, localMac, sizeof(ethernetHeader->source));
    ethernetHeader->etherType = htobe16(ETHERTYPE_IPV4);

    ipv4->versionAndIhl = (IPV4_VERSION << 4) | IPV4_MIN_IHL;
    ipv4->dscpAndEcn = 0;
    ipv4->totalLength = htobe16(sizeof(*ipv4) + sizeof(*udp) + dnsLength);
    ipv4->identification = htobe16(queryId);
    ipv4->flagsAndFragmentOffset = htobe16(IPV4_FLAG_DONT_FRAGMENT);
    ipv4->ttl = IPV4_TTL;
    ipv4->protocol = IPV4_PROTOCOL_UDP;
    ipv4->headerChecksum = 0;
    memcpy(ipv4->sourceIp, localIp, sizeof(ipv4->sourceIp));
    memcpy(ipv4->destinationIp, serverIp, sizeof(ipv4->destinationIp));
    ipv4->headerChecksum = htobe16(calculateChecksum(ipv4, sizeof(*ipv4)));

    udp->sourcePort = htobe16(sourcePort);
    udp->destinationPort = htobe16(UDP_DNS_PORT);
    udp->length = htobe16(sizeof(*udp) + dnsLength);
    udp->checksum = 0;
    memcpy(payload, dnsPayload, dnsLength);
    udp->checksum = htobe16(calculateTransportChecksum(localIp, serverIp,
            IPV4_PROTOCOL_UDP, udp, sizeof(*udp) + dnsLength));

    if (write(fd, frame, frameLength) < 0) {
        err(1, "write");
    }
}

static void sendEchoRequest(int fd, const uint8_t localMac[6],
        const uint8_t nextHopMac[6], const uint8_t localIp[4],
        const uint8_t targetIp[4], uint16_t identifier, uint16_t sequence,
        size_t payloadSize) {
    uint8_t frame[NET_MAX_FRAME_SIZE];
    const size_t frameLength = sizeof(struct EthernetHeader) +
            sizeof(struct Ipv4Header) + sizeof(struct IcmpEchoHeader) +
            payloadSize;

    struct EthernetHeader* ethernetHeader = (struct EthernetHeader*) frame;
    struct Ipv4Header* ipv4 = (struct Ipv4Header*) (frame +
            sizeof(*ethernetHeader));
    struct IcmpEchoHeader* icmp = (struct IcmpEchoHeader*) ((uint8_t*) ipv4 +
            sizeof(*ipv4));
    uint8_t* payload = (uint8_t*) (icmp + 1);

    memcpy(ethernetHeader->destination, nextHopMac,
            sizeof(ethernetHeader->destination));
    memcpy(ethernetHeader->source, localMac, sizeof(ethernetHeader->source));
    ethernetHeader->etherType = htobe16(ETHERTYPE_IPV4);

    ipv4->versionAndIhl = (IPV4_VERSION << 4) | IPV4_MIN_IHL;
    ipv4->dscpAndEcn = 0;
    ipv4->totalLength = htobe16(sizeof(*ipv4) + sizeof(*icmp) + payloadSize);
    ipv4->identification = htobe16(sequence);
    ipv4->flagsAndFragmentOffset = htobe16(IPV4_FLAG_DONT_FRAGMENT);
    ipv4->ttl = IPV4_TTL;
    ipv4->protocol = IPV4_PROTOCOL_ICMP;
    ipv4->headerChecksum = 0;
    memcpy(ipv4->sourceIp, localIp, sizeof(ipv4->sourceIp));
    memcpy(ipv4->destinationIp, targetIp, sizeof(ipv4->destinationIp));
    ipv4->headerChecksum = htobe16(calculateChecksum(ipv4, sizeof(*ipv4)));

    icmp->type = ICMP_TYPE_ECHO_REQUEST;
    icmp->code = 0;
    icmp->checksum = 0;
    icmp->identifier = htobe16(identifier);
    icmp->sequence = htobe16(sequence);

    for (size_t i = 0; i < payloadSize; i++) {
        payload[i] = (uint8_t) i;
    }

    icmp->checksum = htobe16(calculateChecksum(icmp, sizeof(*icmp) +
            payloadSize));

    if (write(fd, frame, frameLength) < 0) {
        err(1, "write");
    }
}

static bool sameSubnet(const uint8_t address1[4], const uint8_t address2[4],
        const uint8_t netmask[4]) {
    for (size_t i = 0; i < 4; i++) {
        if ((address1[i] & netmask[i]) != (address2[i] & netmask[i])) {
            return false;
        }
    }
    return true;
}

static bool skipDnsName(const uint8_t* message, size_t messageLength,
        size_t* offset) {
    while (*offset < messageLength) {
        uint8_t length = message[*offset];
        if ((length & 0xC0) == 0xC0) {
            if (*offset + 2 > messageLength) return false;
            *offset += 2;
            return true;
        }
        if (length & 0xC0) return false;

        (*offset)++;
        if (length == 0) return true;
        if (*offset + length > messageLength) return false;
        *offset += length;
    }

    return false;
}

static void sleepMilliseconds(int milliseconds) {
    struct timespec request;
    request.tv_sec = milliseconds / 1000;
    request.tv_nsec = (milliseconds % 1000) * 1000000L;

    while (!interrupted && nanosleep(&request, &request) < 0 &&
            errno == EINTR) {
    }
}

static bool waitForDnsReply(int fd, const uint8_t localIp[4],
        const uint8_t serverIp[4], uint16_t sourcePort, uint16_t queryId,
        unsigned int timeoutMs, uint8_t resolvedIp[4]) {
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    uint8_t frame[NET_MAX_FRAME_SIZE];
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (!interrupted) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = elapsedNanoseconds(&start, &now);
        int64_t remainingNs = (int64_t) timeoutMs * 1000000 - elapsed;
        if (remainingNs <= 0) return false;

        int timeout = (int) ((remainingNs + 999999) / 1000000);
        int result = poll(&pfd, 1, timeout);
        if (result < 0) {
            if (errno == EINTR) continue;
            err(1, "poll");
        }
        if (result == 0) return false;
        if (!(pfd.revents & (POLLIN | POLLRDNORM))) continue;

        ssize_t bytesRead = read(fd, frame, sizeof(frame));
        if (bytesRead < 0) {
            if (errno == EINTR) continue;
            err(1, "read");
        }

        if (handleDnsResponse(frame, (size_t) bytesRead, localIp, serverIp,
                sourcePort, queryId, resolvedIp)) {
            return true;
        }
    }

    return false;
}

static bool waitForEchoReply(int fd, const uint8_t localIp[4],
        const uint8_t targetIp[4], uint16_t identifier, uint16_t sequence,
        unsigned int timeoutMs, uint8_t* ttl, size_t* icmpBytes) {
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    uint8_t frame[NET_MAX_FRAME_SIZE];
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (!interrupted) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = elapsedNanoseconds(&start, &now);
        int64_t remainingNs = (int64_t) timeoutMs * 1000000 - elapsed;
        if (remainingNs <= 0) return false;

        int timeout = (int) ((remainingNs + 999999) / 1000000);
        int result = poll(&pfd, 1, timeout);
        if (result < 0) {
            if (errno == EINTR) continue;
            err(1, "poll");
        }
        if (result == 0) return false;
        if (!(pfd.revents & (POLLIN | POLLRDNORM))) continue;

        ssize_t bytesRead = read(fd, frame, sizeof(frame));
        if (bytesRead < 0) {
            if (errno == EINTR) continue;
            err(1, "read");
        }

        if (handleEchoReply(frame, (size_t) bytesRead, localIp, targetIp,
                identifier, sequence, ttl, icmpBytes)) {
            return true;
        }
    }

    return false;
}
