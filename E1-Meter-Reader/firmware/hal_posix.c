/*
 * hal_posix.c — Linux simulator backend.
 *
 * The "serial link" is a loopback TCP socket: the firmware listens on
 * 127.0.0.1:$E1_SIM_PORT (default 5555; 0 = pick a free port and print
 * it).  The host viewer connects with --sim 127.0.0.1:PORT.  A
 * disconnected host can simply reconnect; the firmware keeps running.
 *
 * User button = `kill -USR1 <pid>`.
 */
#define _POSIX_C_SOURCE 200809L

#include "hal.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int s_listen_fd = -1;
static int s_client_fd = -1;
static struct timespec s_t0;
static volatile sig_atomic_t s_button;

static void on_sigusr1(int sig)
{
    (void)sig;
    s_button = 1;
}

static void accept_client(void)
{
    fprintf(stderr, "[hal_posix] waiting for host connection...\n");
    fflush(stderr);
    for (;;) {
        int fd = accept(s_listen_fd, NULL, NULL);

        if (fd >= 0) {
            int one = 1;

            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            s_client_fd = fd;
            fprintf(stderr, "[hal_posix] host connected\n");
            fflush(stderr);
            return;
        }
        if (errno == EINTR)
            continue;
        perror("[hal_posix] accept");
        exit(1);
    }
}

void hal_init(void)
{
    struct sigaction sa;
    struct sockaddr_in addr;
    struct sockaddr_in got;
    socklen_t gl = sizeof got;
    const char *pstr = getenv("E1_SIM_PORT");
    int port = pstr ? atoi(pstr) : 5555;
    int one = 1;

    signal(SIGPIPE, SIG_IGN);
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sigusr1;
    sigaction(SIGUSR1, &sa, NULL);

    clock_gettime(CLOCK_MONOTONIC, &s_t0);

    s_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s_listen_fd < 0) {
        perror("[hal_posix] socket");
        exit(1);
    }
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);
    if (bind(s_listen_fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        perror("[hal_posix] bind");
        exit(1);
    }
    if (listen(s_listen_fd, 1) < 0) {
        perror("[hal_posix] listen");
        exit(1);
    }
    getsockname(s_listen_fd, (struct sockaddr *)&got, &gl);
    fprintf(stderr,
            "[hal_posix] listening on 127.0.0.1:%d  (button: kill -USR1 %d)\n",
            (int)ntohs(got.sin_port), (int)getpid());
    fflush(stderr);

    accept_client();
}

int hal_serial_read(uint8_t *buf, int len, int timeout_ms)
{
    for (;;) {
        struct pollfd pf;
        int pr;
        ssize_t r;

        pf.fd = s_client_fd;
        pf.events = POLLIN;
        pr = poll(&pf, 1, timeout_ms);
        if (pr < 0) {
            if (errno == EINTR)
                return 0; /* let the superloop see the button promptly */
            perror("[hal_posix] poll");
            exit(1);
        }
        if (pr == 0)
            return 0;

        r = read(s_client_fd, buf, (size_t)len);
        if (r > 0)
            return (int)r;
        if (r < 0 && errno == EINTR)
            continue;
        /* EOF or hard error: host went away — wait for a new one */
        fprintf(stderr, "[hal_posix] host disconnected\n");
        close(s_client_fd);
        s_client_fd = -1;
        accept_client();
        return 0;
    }
}

int hal_serial_write(const uint8_t *buf, int len)
{
    int done = 0;

    while (done < len) {
        ssize_t w = write(s_client_fd, buf + done, (size_t)(len - done));

        if (w > 0) {
            done += (int)w;
            continue;
        }
        if (w < 0 && errno == EINTR)
            continue;
        /* broken pipe: drop the rest, wait for a new host */
        fprintf(stderr, "[hal_posix] host disconnected (write)\n");
        close(s_client_fd);
        s_client_fd = -1;
        accept_client();
        return done;
    }
    return done;
}

uint32_t hal_millis(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec - s_t0.tv_sec) * 1000 +
                      (ts.tv_nsec - s_t0.tv_nsec) / 1000000);
}

uint32_t hal_micros(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec - s_t0.tv_sec) * 1000000 +
                      (ts.tv_nsec - s_t0.tv_nsec) / 1000);
}

bool hal_button_pressed(void)
{
    if (s_button) {
        s_button = 0;
        fprintf(stderr, "[hal_posix] button press (SIGUSR1)\n");
        fflush(stderr);
        return true;
    }
    return false;
}
