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

/* utils/dhcp.c
 * DHCP client over raw Ethernet devices.
 */

#include "utils.h"
#include <dirent.h>
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
#include <sys/stat.h>
#include <lunix/net.h>

#define DEFAULT_DEVICE "/dev/eth0"
#define DEFAULT_TIMEOUT_MS 4000
#define DEFAULT_ATTEMPTS 3

#define ETHERTYPE_IPV4 0x0800

#define IPV4_VERSION 4
#define IPV4_MIN_IHL 5
#define IPV4_PROTOCOL_UDP 17
#define IPV4_TTL 64
#define IPV4_FLAG_DONT_FRAGMENT 0x4000
#define IPV4_FLAG_MORE_FRAGMENTS 0x2000
#define IPV4_FRAGMENT_OFFSET_MASK 0x1FFF

#define DHCP_SERVER_PORT 67
#define DHCP_CLIENT_PORT 68
#define DHCP_FLAGS_BROADCAST 0x8000
#define DHCP_MAGIC 0x63825363U

#define DHCPDISCOVER 1
#define DHCPOFFER 2
#define DHCPREQUEST 3
#define DHCPACK 5
#define DHCPNAK 6

#define DHCP_OPTION_SUBNET_MASK 1
#define DHCP_OPTION_ROUTER 3
#define DHCP_OPTION_DNS 6
#define DHCP_OPTION_HOST_NAME 12
#define DHCP_OPTION_REQUESTED_IP 50
#define DHCP_OPTION_LEASE_TIME 51
#define DHCP_OPTION_MESSAGE_TYPE 53
#define DHCP_OPTION_SERVER_ID 54
#define DHCP_OPTION_MAX_MESSAGE_SIZE 57
#define DHCP_OPTION_PARAMETER_LIST 55
#define DHCP_OPTION_RENEWAL_TIME 58
#define DHCP_OPTION_REBINDING_TIME 59
#define DHCP_OPTION_CLIENT_IDENTIFIER 61
#define DHCP_OPTION_END 255

#define DEVICE_PATH_MAX 64
#define DHCP_MAX_MESSAGE_SIZE 1500
#define NETWORK_CONFIG_DIR "/run/network"
#define NETWORK_CONFIG_PATH_MAX 256

struct EthernetHeader {
    uint8_t destination[6];
    uint8_t source[6];
    uint16_t etherType;
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

struct BootpHeader {
    uint8_t op;
    uint8_t hardwareType;
    uint8_t hardwareLength;
    uint8_t hops;
    uint32_t transactionId;
    uint16_t seconds;
    uint16_t flags;
    uint8_t clientIp[4];
    uint8_t yourIp[4];
    uint8_t serverIp[4];
    uint8_t gatewayIp[4];
    uint8_t clientHardwareAddress[16];
    uint8_t serverName[64];
    uint8_t bootFile[128];
} __attribute__((packed));

struct DhcpLease {
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t nameserver[4];
    uint8_t serverId[4];
    bool haveNetmask;
    bool haveGateway;
    bool haveNameserver;
    bool haveServerId;
    bool haveLeaseTime;
    bool haveRenewalTime;
    bool haveRebindingTime;
    uint32_t leaseTime;
    uint32_t renewalTime;
    uint32_t rebindingTime;
};

static volatile sig_atomic_t interrupted = 0;

static uint32_t addChecksumWords(uint32_t sum, const void* buffer,
        size_t length);
static uint16_t calculateChecksum(const void* buffer, size_t length);
static bool acquireDhcpLease(int fd, const uint8_t localMac[6],
        unsigned int attempts, unsigned int timeoutMs,
        struct DhcpLease* finalLease);
static int64_t elapsedNanoseconds(const struct timespec* start,
        const struct timespec* end);
static void formatIpv4(const uint8_t address[4], char buffer[16]);
static bool getIpv4Payload(const uint8_t* frame, size_t frameLength,
        uint8_t expectedProtocol, const uint8_t** payload,
        size_t* payloadLength);
static void handleSignal(int signalNumber);
static bool isEthernetDeviceName(const char* name);
static void mergeLease(struct DhcpLease* destination,
        const struct DhcpLease* source);
static long parseLong(const char* string, const char* option, long min,
        long max);
static bool parseDhcpMessage(const uint8_t* frame, size_t frameLength,
        uint32_t transactionId, const uint8_t localMac[6], int* messageType,
        struct DhcpLease* lease);
static void sendDhcpMessage(int fd, const uint8_t localMac[6],
        uint32_t transactionId, int messageType, const uint8_t requestedIp[4],
        const uint8_t serverId[4]);
static bool waitForDhcpMessage(int fd, uint32_t transactionId,
        const uint8_t localMac[6], unsigned int timeoutMs, int* messageType,
        struct DhcpLease* lease);
static bool writeLeaseConfig(const char* device, const struct DhcpLease* lease,
        char path[NETWORK_CONFIG_PATH_MAX]);

int main(int argc, char* argv[]) {
    static const struct option longopts[] = {
        { "attempts", required_argument, 0, 'a' },
        { "device", required_argument, 0, 'd' },
        { "timeout", required_argument, 0, 't' },
        { "help", no_argument, 0, 0 },
        { "version", no_argument, 0, 1 },
        { 0, 0, 0, 0 }
    };

    const char* device = NULL;
    unsigned int attempts = DEFAULT_ATTEMPTS;
    unsigned int timeoutMs = DEFAULT_TIMEOUT_MS;
    bool deviceSpecified = false;

    int c;
    while ((c = getopt_long(argc, argv, "a:d:t:", longopts, NULL)) != -1) {
        switch (c) {
        case 0:
            return help(argv[0], "[OPTIONS] [DEVICE]\n"
                    "  -a, --attempts=COUNT     DHCP discover/request attempts\n"
                    "  -d, --device=DEVICE      raw ethernet device\n"
                    "  -t, --timeout=MS         timeout per DHCP step in "
                    "milliseconds\n"
                    "      --help               display this help\n"
                    "      --version            display version info\n"
                    "\n"
                    "If no device is specified, dhcp probes link-up /dev/eth*\n"
                    "interfaces until it receives a lease.\n"
                    "\n"
                    "The acquired lease is stored in /run/network/<iface>.conf "
                    "for other\n"
                    "utilities such as ping.");
        case 1:
            return version(argv[0]);
        case 'a':
            attempts = (unsigned int) parseLong(optarg, "--attempts", 1,
                    INT_MAX);
            break;
        case 'd':
            device = optarg;
            deviceSpecified = true;
            break;
        case 't':
            timeoutMs = (unsigned int) parseLong(optarg, "--timeout", 1,
                    INT_MAX);
            break;
        case '?':
            return 1;
        }
    }

    if (optind < argc) {
        if (optind + 1 != argc) {
            errx(1, "extra operand: '%s'", argv[optind + 1]);
        }
        device = argv[optind];
        deviceSpecified = true;
    }

    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = handleSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);

    char selectedDevice[DEVICE_PATH_MAX];
    struct DhcpLease finalLease = {};
    bool gotLease = false;

    if (deviceSpecified) {
        int fd = open(device, O_RDWR);
        if (fd < 0) err(1, "open: '%s'", device);

        struct net_info info;
        if (ioctl(fd, NET_GET_INFO, &info) < 0) {
            err(1, "ioctl(NET_GET_INFO): '%s'", device);
        }
        if (!(info.flags & NET_INFO_LINK_UP)) {
            errx(1, "%s: link is down", device);
        }

        gotLease = acquireDhcpLease(fd, info.mac, attempts, timeoutMs,
                &finalLease);
        close(fd);

        if (!gotLease) {
            errx(1, "failed to acquire DHCP lease on %s", device);
        }

        snprintf(selectedDevice, sizeof(selectedDevice), "%s", device);
    } else {
        DIR* dir = opendir("/dev");
        if (!dir) err(1, "opendir: '/dev'");

        bool sawCandidate = false;
        errno = 0;
        struct dirent* dirent;
        while ((dirent = readdir(dir))) {
            if (!isEthernetDeviceName(dirent->d_name)) continue;

            sawCandidate = true;

            char path[DEVICE_PATH_MAX];
            if (snprintf(path, sizeof(path), "/dev/%s", dirent->d_name) >=
                    (int) sizeof(path)) {
                continue;
            }

            int fd = open(path, O_RDWR);
            if (fd < 0) continue;

            struct net_info info;
            if (ioctl(fd, NET_GET_INFO, &info) < 0 ||
                    !(info.flags & NET_INFO_LINK_UP)) {
                close(fd);
                continue;
            }

            if (acquireDhcpLease(fd, info.mac, attempts, timeoutMs,
                    &finalLease)) {
                snprintf(selectedDevice, sizeof(selectedDevice), "%s", path);
                gotLease = true;
                close(fd);
                break;
            }

            close(fd);
        }

        if (errno != 0) err(1, "readdir: '/dev'");
        closedir(dir);

        if (!gotLease) {
            errx(1, sawCandidate ? "failed to acquire DHCP lease on any "
                    "/dev/eth* interface" : "no /dev/eth* interfaces found");
        }
    }

    char configPath[NETWORK_CONFIG_PATH_MAX];
    if (!writeLeaseConfig(selectedDevice, &finalLease, configPath)) {
        err(1, "writing lease config");
    }

    char ip[16];
    char netmask[16];
    char gateway[16];
    char nameserver[16];
    formatIpv4(finalLease.ip, ip);
    formatIpv4(finalLease.netmask, netmask);
    formatIpv4(finalLease.gateway, gateway);
    formatIpv4(finalLease.nameserver, nameserver);

    printf("%s: leased %s", selectedDevice, ip);
    if (finalLease.haveNetmask) printf(" netmask %s", netmask);
    if (finalLease.haveGateway) printf(" gateway %s", gateway);
    if (finalLease.haveNameserver) printf(" dns %s", nameserver);
    if (finalLease.haveLeaseTime) {
        printf(" lease %" PRIu32 "s", finalLease.leaseTime);
    }
    putchar('\n');
    printf("saved %s\n", configPath);

    return 0;
}

static bool acquireDhcpLease(int fd, const uint8_t localMac[6],
        unsigned int attempts, unsigned int timeoutMs,
        struct DhcpLease* finalLease) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint32_t baseTransactionId = (uint32_t) now.tv_nsec ^
            (uint32_t) now.tv_sec ^ (uint32_t) getpid();

    for (unsigned int attempt = 0; attempt < attempts && !interrupted;
            attempt++) {
        uint32_t transactionId = baseTransactionId + attempt;
        sendDhcpMessage(fd, localMac, transactionId, DHCPDISCOVER, NULL, NULL);

        int messageType;
        struct DhcpLease offer = {};
        if (!waitForDhcpMessage(fd, transactionId, localMac, timeoutMs,
                &messageType, &offer) || messageType != DHCPOFFER) {
            continue;
        }

        if (!offer.haveServerId) {
            warnx("ignoring DHCP offer without server identifier");
            continue;
        }

        sendDhcpMessage(fd, localMac, transactionId, DHCPREQUEST, offer.ip,
                offer.serverId);

        struct DhcpLease ack = {};
        if (!waitForDhcpMessage(fd, transactionId, localMac, timeoutMs,
                &messageType, &ack)) {
            continue;
        }

        if (messageType == DHCPNAK) {
            warnx("received DHCPNAK");
            continue;
        }
        if (messageType != DHCPACK) continue;

        mergeLease(&ack, &offer);
        *finalLease = ack;
        return true;
    }

    return false;
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

static int64_t elapsedNanoseconds(const struct timespec* start,
        const struct timespec* end) {
    return (end->tv_sec - start->tv_sec) * 1000000000LL +
            (end->tv_nsec - start->tv_nsec);
}

static void formatIpv4(const uint8_t address[4], char buffer[16]) {
    snprintf(buffer, 16, "%u.%u.%u.%u", address[0], address[1], address[2],
            address[3]);
}

static bool getIpv4Payload(const uint8_t* frame, size_t frameLength,
        uint8_t expectedProtocol, const uint8_t** payload,
        size_t* payloadLength) {
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

    *payload = ipBuffer + headerLength;
    *payloadLength = totalLength - headerLength;
    return true;
}

static void handleSignal(int signalNumber) {
    (void) signalNumber;
    interrupted = 1;
}

static bool isEthernetDeviceName(const char* name) {
    if (strncmp(name, "eth", 3) != 0 || name[3] == '\0') return false;

    for (const char* p = name + 3; *p; p++) {
        if (*p < '0' || *p > '9') return false;
    }

    return true;
}

static void mergeLease(struct DhcpLease* destination,
        const struct DhcpLease* source) {
    if (!destination->haveNetmask && source->haveNetmask) {
        memcpy(destination->netmask, source->netmask, 4);
        destination->haveNetmask = true;
    }
    if (!destination->haveGateway && source->haveGateway) {
        memcpy(destination->gateway, source->gateway, 4);
        destination->haveGateway = true;
    }
    if (!destination->haveNameserver && source->haveNameserver) {
        memcpy(destination->nameserver, source->nameserver, 4);
        destination->haveNameserver = true;
    }
    if (!destination->haveServerId && source->haveServerId) {
        memcpy(destination->serverId, source->serverId, 4);
        destination->haveServerId = true;
    }
    if (!destination->haveLeaseTime && source->haveLeaseTime) {
        destination->leaseTime = source->leaseTime;
        destination->haveLeaseTime = true;
    }
    if (!destination->haveRenewalTime && source->haveRenewalTime) {
        destination->renewalTime = source->renewalTime;
        destination->haveRenewalTime = true;
    }
    if (!destination->haveRebindingTime && source->haveRebindingTime) {
        destination->rebindingTime = source->rebindingTime;
        destination->haveRebindingTime = true;
    }
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

static bool parseDhcpMessage(const uint8_t* frame, size_t frameLength,
        uint32_t transactionId, const uint8_t localMac[6], int* messageType,
        struct DhcpLease* lease) {
    const uint8_t* payload;
    size_t payloadLength;
    if (!getIpv4Payload(frame, frameLength, IPV4_PROTOCOL_UDP, &payload,
            &payloadLength)) {
        return false;
    }

    if (payloadLength < sizeof(struct UdpHeader) + sizeof(struct BootpHeader) +
            sizeof(uint32_t)) {
        return false;
    }

    const struct UdpHeader* udp = (const struct UdpHeader*) payload;
    uint16_t udpLength = be16toh(udp->length);
    if (be16toh(udp->sourcePort) != DHCP_SERVER_PORT ||
            be16toh(udp->destinationPort) != DHCP_CLIENT_PORT ||
            udpLength < sizeof(*udp) + sizeof(struct BootpHeader) +
            sizeof(uint32_t) || udpLength > payloadLength) {
        return false;
    }

    const uint8_t* dhcpBuffer = payload + sizeof(*udp);
    size_t dhcpLength = udpLength - sizeof(*udp);
    const struct BootpHeader* bootp = (const struct BootpHeader*) dhcpBuffer;
    if (bootp->op != 2 || bootp->hardwareType != 1 ||
            bootp->hardwareLength != 6 ||
            be32toh(bootp->transactionId) != transactionId ||
            memcmp(bootp->clientHardwareAddress, localMac, 6) != 0) {
        return false;
    }

    uint32_t magic = be32toh(*(const uint32_t*) (dhcpBuffer +
            sizeof(*bootp)));
    if (magic != DHCP_MAGIC) return false;

    memset(lease, 0, sizeof(*lease));
    memcpy(lease->ip, bootp->yourIp, 4);

    size_t offset = sizeof(*bootp) + sizeof(uint32_t);
    *messageType = 0;
    while (offset < dhcpLength) {
        uint8_t option = dhcpBuffer[offset++];
        if (option == 0) continue;
        if (option == DHCP_OPTION_END) break;
        if (offset >= dhcpLength) return false;

        uint8_t length = dhcpBuffer[offset++];
        if (offset + length > dhcpLength) return false;

        switch (option) {
        case DHCP_OPTION_MESSAGE_TYPE:
            if (length == 1) *messageType = dhcpBuffer[offset];
            break;
        case DHCP_OPTION_SUBNET_MASK:
            if (length >= 4) {
                memcpy(lease->netmask, dhcpBuffer + offset, 4);
                lease->haveNetmask = true;
            }
            break;
        case DHCP_OPTION_ROUTER:
            if (length >= 4) {
                memcpy(lease->gateway, dhcpBuffer + offset, 4);
                lease->haveGateway = true;
            }
            break;
        case DHCP_OPTION_DNS:
            if (length >= 4) {
                memcpy(lease->nameserver, dhcpBuffer + offset, 4);
                lease->haveNameserver = true;
            }
            break;
        case DHCP_OPTION_SERVER_ID:
            if (length >= 4) {
                memcpy(lease->serverId, dhcpBuffer + offset, 4);
                lease->haveServerId = true;
            }
            break;
        case DHCP_OPTION_LEASE_TIME:
            if (length == 4) {
                lease->leaseTime = be32toh(*(const uint32_t*)
                        (dhcpBuffer + offset));
                lease->haveLeaseTime = true;
            }
            break;
        case DHCP_OPTION_RENEWAL_TIME:
            if (length == 4) {
                lease->renewalTime = be32toh(*(const uint32_t*)
                        (dhcpBuffer + offset));
                lease->haveRenewalTime = true;
            }
            break;
        case DHCP_OPTION_REBINDING_TIME:
            if (length == 4) {
                lease->rebindingTime = be32toh(*(const uint32_t*)
                        (dhcpBuffer + offset));
                lease->haveRebindingTime = true;
            }
            break;
        }

        offset += length;
    }

    if (!lease->haveServerId && memcmp(bootp->serverIp, "\0\0\0\0", 4) != 0) {
        memcpy(lease->serverId, bootp->serverIp, 4);
        lease->haveServerId = true;
    }

    return *messageType != 0;
}

static void sendDhcpMessage(int fd, const uint8_t localMac[6],
        uint32_t transactionId, int messageType, const uint8_t requestedIp[4],
        const uint8_t serverId[4]) {
    uint8_t frame[NET_MAX_FRAME_SIZE];
    uint8_t* payload;
    size_t optionsLength = 0;
    static const char hostName[] = "lunix";

    struct EthernetHeader* ethernetHeader = (struct EthernetHeader*) frame;
    struct Ipv4Header* ipv4 = (struct Ipv4Header*) (frame +
            sizeof(*ethernetHeader));
    struct UdpHeader* udp = (struct UdpHeader*) ((uint8_t*) ipv4 +
            sizeof(*ipv4));
    struct BootpHeader* bootp = (struct BootpHeader*) (udp + 1);
    uint8_t* magic = (uint8_t*) (bootp + 1);
    payload = magic + sizeof(uint32_t);

    memset(ethernetHeader->destination, 0xFF,
            sizeof(ethernetHeader->destination));
    memcpy(ethernetHeader->source, localMac, sizeof(ethernetHeader->source));
    ethernetHeader->etherType = htobe16(ETHERTYPE_IPV4);

    ipv4->versionAndIhl = (IPV4_VERSION << 4) | IPV4_MIN_IHL;
    ipv4->dscpAndEcn = 0;
    ipv4->identification = htobe16(transactionId);
    ipv4->flagsAndFragmentOffset = htobe16(IPV4_FLAG_DONT_FRAGMENT);
    ipv4->ttl = IPV4_TTL;
    ipv4->protocol = IPV4_PROTOCOL_UDP;
    ipv4->headerChecksum = 0;
    memset(ipv4->sourceIp, 0, sizeof(ipv4->sourceIp));
    memset(ipv4->destinationIp, 0xFF, sizeof(ipv4->destinationIp));

    udp->sourcePort = htobe16(DHCP_CLIENT_PORT);
    udp->destinationPort = htobe16(DHCP_SERVER_PORT);
    udp->checksum = 0;

    memset(bootp, 0, sizeof(*bootp));
    bootp->op = 1;
    bootp->hardwareType = 1;
    bootp->hardwareLength = 6;
    bootp->transactionId = htobe32(transactionId);
    bootp->flags = htobe16(DHCP_FLAGS_BROADCAST);
    memcpy(bootp->clientHardwareAddress, localMac, 6);

    *(uint32_t*) magic = htobe32(DHCP_MAGIC);

    payload[optionsLength++] = DHCP_OPTION_MESSAGE_TYPE;
    payload[optionsLength++] = 1;
    payload[optionsLength++] = messageType;

    payload[optionsLength++] = DHCP_OPTION_CLIENT_IDENTIFIER;
    payload[optionsLength++] = 1 + 6;
    payload[optionsLength++] = 1;
    memcpy(payload + optionsLength, localMac, 6);
    optionsLength += 6;

    payload[optionsLength++] = DHCP_OPTION_MAX_MESSAGE_SIZE;
    payload[optionsLength++] = 2;
    payload[optionsLength++] = (uint8_t) (DHCP_MAX_MESSAGE_SIZE >> 8);
    payload[optionsLength++] = (uint8_t) DHCP_MAX_MESSAGE_SIZE;

    payload[optionsLength++] = DHCP_OPTION_HOST_NAME;
    payload[optionsLength++] = sizeof(hostName) - 1;
    memcpy(payload + optionsLength, hostName, sizeof(hostName) - 1);
    optionsLength += sizeof(hostName) - 1;

    if (messageType == DHCPREQUEST && requestedIp && serverId) {
        payload[optionsLength++] = DHCP_OPTION_REQUESTED_IP;
        payload[optionsLength++] = 4;
        memcpy(payload + optionsLength, requestedIp, 4);
        optionsLength += 4;

        payload[optionsLength++] = DHCP_OPTION_SERVER_ID;
        payload[optionsLength++] = 4;
        memcpy(payload + optionsLength, serverId, 4);
        optionsLength += 4;
    }

    payload[optionsLength++] = DHCP_OPTION_PARAMETER_LIST;
    payload[optionsLength++] = 6;
    payload[optionsLength++] = DHCP_OPTION_SUBNET_MASK;
    payload[optionsLength++] = DHCP_OPTION_ROUTER;
    payload[optionsLength++] = DHCP_OPTION_DNS;
    payload[optionsLength++] = DHCP_OPTION_LEASE_TIME;
    payload[optionsLength++] = DHCP_OPTION_RENEWAL_TIME;
    payload[optionsLength++] = DHCP_OPTION_REBINDING_TIME;
    payload[optionsLength++] = DHCP_OPTION_END;

    size_t udpPayloadLength = sizeof(*bootp) + sizeof(uint32_t) + optionsLength;
    udp->length = htobe16(sizeof(*udp) + udpPayloadLength);
    ipv4->totalLength = htobe16(sizeof(*ipv4) + sizeof(*udp) +
            udpPayloadLength);
    ipv4->headerChecksum = htobe16(calculateChecksum(ipv4, sizeof(*ipv4)));
    // IPv4 DHCP accepts a zero UDP checksum and many implementations use it.
    // Keeping it zero avoids servers dropping packets if they dislike a
    // client-generated checksum here.
    udp->checksum = 0;

    size_t frameLength = sizeof(*ethernetHeader) + sizeof(*ipv4) +
            sizeof(*udp) + udpPayloadLength;
    if (write(fd, frame, frameLength) < 0) {
        err(1, "write");
    }
}

static bool waitForDhcpMessage(int fd, uint32_t transactionId,
        const uint8_t localMac[6], unsigned int timeoutMs, int* messageType,
        struct DhcpLease* lease) {
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

        if (parseDhcpMessage(frame, (size_t) bytesRead, transactionId,
                localMac, messageType, lease)) {
            return true;
        }
    }

    return false;
}

static bool writeLeaseConfig(const char* device, const struct DhcpLease* lease,
        char path[NETWORK_CONFIG_PATH_MAX]) {
    const char* interfaceName = strrchr(device, '/');
    interfaceName = interfaceName ? interfaceName + 1 : device;
    if (!*interfaceName) {
        errno = EINVAL;
        return false;
    }

    if (mkdir(NETWORK_CONFIG_DIR, 0755) < 0 && errno != EEXIST) {
        return false;
    }

    if (snprintf(path, NETWORK_CONFIG_PATH_MAX, "%s/%s.conf",
            NETWORK_CONFIG_DIR, interfaceName) >= NETWORK_CONFIG_PATH_MAX) {
        errno = ENAMETOOLONG;
        return false;
    }

    FILE* file = fopen(path, "w");
    if (!file) return false;

    char buffer[16];
    formatIpv4(lease->ip, buffer);
    fprintf(file, "ip=%s\n", buffer);

    if (lease->haveNetmask) {
        formatIpv4(lease->netmask, buffer);
        fprintf(file, "netmask=%s\n", buffer);
    }
    if (lease->haveGateway) {
        formatIpv4(lease->gateway, buffer);
        fprintf(file, "gateway=%s\n", buffer);
    }
    if (lease->haveNameserver) {
        formatIpv4(lease->nameserver, buffer);
        fprintf(file, "nameserver=%s\n", buffer);
    }
    if (lease->haveServerId) {
        formatIpv4(lease->serverId, buffer);
        fprintf(file, "server=%s\n", buffer);
    }
    if (lease->haveLeaseTime) {
        fprintf(file, "lease_time=%" PRIu32 "\n", lease->leaseTime);
    }
    if (lease->haveRenewalTime) {
        fprintf(file, "renewal_time=%" PRIu32 "\n", lease->renewalTime);
    }
    if (lease->haveRebindingTime) {
        fprintf(file, "rebinding_time=%" PRIu32 "\n", lease->rebindingTime);
    }

    if (fclose(file) < 0) return false;
    return true;
}
