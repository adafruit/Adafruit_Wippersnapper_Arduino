#ifndef NANOSHA256_H
#define NANOSHA256_H

/**
 * © 2026-present oopsio
 * nanosha256 - Security Refactored Version
 * 
 * High-performance, memory-safe, single-header SHA-256 library.
 * Features: NULL protection, overflow guards, and secure memory cleanup.
 */

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#if defined(__GNUC__) || defined(__clang__)
    #define SHA256_BSWAP32(x) __builtin_bswap32(x)
    #define SHA256_BSWAP64(x) __builtin_bswap64(x)
#elif defined(_MSC_VER)
    #include <stdlib.h>
    #define SHA256_BSWAP32(x) _byteswap_ulong(x)
    #define SHA256_BSWAP64(x) _byteswap_uint64(x)
#else
    static inline uint32_t SHA256_BSWAP32(uint32_t x) {
        return ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24));
    }
    static inline uint64_t SHA256_BSWAP64(uint64_t x) {
        return (((uint64_t)SHA256_BSWAP32((uint32_t)x) << 32) | SHA256_BSWAP32((uint32_t)(x >> 32)));
    }
#endif

/* Detect Little Endian for fast block loading */
#if (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) || \
    defined(_WIN32) || defined(__i386__) || defined(__x86_64__) || defined(__ARMEL__)
    #define SHA256_LITTLE_ENDIAN
#endif

#ifdef SHA256_LITTLE_ENDIAN
    #define BE32(x) SHA256_BSWAP32(x)
    #define BE64(x) SHA256_BSWAP64(x)
#else
    #define BE32(x) (x)
    #define BE64(x) (x)
#endif


#define SHA256_ROTR(x, n)  (((x) >> (n)) | ((x) << (32 - (n))))
#define SHA256_CH(x, y, z) (((x) & (y)) ^ (~(x) & (z)))
#define SHA256_MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define SHA256_EP0(x) (SHA256_ROTR(x, 2) ^ SHA256_ROTR(x, 13) ^ SHA256_ROTR(x, 22))
#define SHA256_EP1(x) (SHA256_ROTR(x, 6) ^ SHA256_ROTR(x, 11) ^ SHA256_ROTR(x, 25))
#define SHA256_SIG0(x) (SHA256_ROTR(x, 7) ^ SHA256_ROTR(x, 18) ^ ((x) >> 3))
#define SHA256_SIG1(x) (SHA256_ROTR(x, 17) ^ SHA256_ROTR(x, 19) ^ ((x) >> 10))

#define SHA256_ROUND(a, b, c, d, e, f, g, h, k, w) do { \
    uint32_t t1 = (h) + SHA256_EP1(e) + SHA256_CH(e, f, g) + (k) + (w); \
    uint32_t t2 = SHA256_EP0(a) + SHA256_MAJ(a, b, c); \
    (d) += t1; \
    (h) = t1 + t2; \
} while (0)

static const uint32_t SHA256_K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};


/**
 * Initializes the SHA-256 state. Returns 0 on success, -1 on NULL.
 */
static inline int sha256_init(uint32_t state[8]) {
    if (!state) return -1;
    state[0] = 0x6a09e667;
    state[1] = 0xbb67ae85;
    state[2] = 0x3c6ef372;
    state[3] = 0xa54ff53a;
    state[4] = 0x510e527f;
    state[5] = 0x9b05688c;
    state[6] = 0x1f83d9ab;
    state[7] = 0x5be0cd19;
    return 0;
}

/**
 * Processes a single 64-byte block. Returns 0 on success.
 */
static inline int sha256_transform(uint32_t state[8], const uint8_t data[64]) {
    if (!state || !data) return -1;
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
    uint32_t w[64];
    
    /* Alignment Safety: Use memcpy to load data into the work array */
    for (int i = 0; i < 16; i++) {
        uint32_t val;
        memcpy(&val, data + i * 4, 4);
        w[i] = BE32(val);
    }

    for (int i = 16; i < 64; i++) {
        w[i] = SHA256_SIG1(w[i - 2]) + w[i - 7] + SHA256_SIG0(w[i - 15]) + w[i - 16];
    }

    /* Unrolled Compression Rounds */
    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[0],  w[0]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[1],  w[1]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[2],  w[2]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[3],  w[3]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[4],  w[4]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[5],  w[5]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[6],  w[6]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[7],  w[7]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[8],  w[8]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[9],  w[9]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[10], w[10]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[11], w[11]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[12], w[12]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[13], w[13]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[14], w[14]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[15], w[15]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[16], w[16]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[17], w[17]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[18], w[18]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[19], w[19]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[20], w[20]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[21], w[21]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[22], w[22]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[23], w[23]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[24], w[24]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[25], w[25]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[26], w[26]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[27], w[27]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[28], w[28]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[29], w[29]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[30], w[30]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[31], w[31]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[32], w[32]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[33], w[33]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[34], w[34]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[35], w[35]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[36], w[36]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[37], w[37]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[38], w[38]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[39], w[39]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[40], w[40]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[41], w[41]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[42], w[42]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[43], w[43]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[44], w[44]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[45], w[45]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[46], w[46]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[47], w[47]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[48], w[48]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[49], w[49]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[50], w[50]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[51], w[51]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[52], w[52]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[53], w[53]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[54], w[54]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[55], w[55]);

    SHA256_ROUND(a, b, c, d, e, f, g, h, SHA256_K[56], w[56]);
    SHA256_ROUND(h, a, b, c, d, e, f, g, SHA256_K[57], w[57]);
    SHA256_ROUND(g, h, a, b, c, d, e, f, SHA256_K[58], w[58]);
    SHA256_ROUND(f, g, h, a, b, c, d, e, SHA256_K[59], w[59]);
    SHA256_ROUND(e, f, g, h, a, b, c, d, SHA256_K[60], w[60]);
    SHA256_ROUND(d, e, f, g, h, a, b, c, SHA256_K[61], w[61]);
    SHA256_ROUND(c, d, e, f, g, h, a, b, SHA256_K[62], w[62]);
    SHA256_ROUND(b, c, d, e, f, g, h, a, SHA256_K[63], w[63]);

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
    return 0;
}



typedef struct {
    uint32_t state[8];
    uint64_t total_len;
    uint8_t buffer[64];
} SHA256_CTX;

static inline int SHA256_Init(SHA256_CTX *ctx) {
    if (!ctx) return -1;
    sha256_init(ctx->state);
    ctx->total_len = 0;
    memset(ctx->buffer, 0, 64);
    return 0;
}

static inline int SHA256_Update(SHA256_CTX *ctx, const void *data, size_t len) {
    if (!ctx || !data) return -1;
    
    /* Integer Overflow Check */
    if (len > (uint64_t)-1 - ctx->total_len) return -2;

    const uint8_t *p = (const uint8_t *)data;
    uint32_t left = (uint32_t)(ctx->total_len & 0x3F);
    uint32_t fill = 64 - left;

    ctx->total_len += len;

    if (left && len >= fill) {
        /* Buffer Bounds: left + fill is exactly 64 */
        memcpy(ctx->buffer + left, p, fill);
        sha256_transform(ctx->state, ctx->buffer);
        p += fill;
        len -= fill;
        left = 0;
    }

    while (len >= 64) {
        sha256_transform(ctx->state, p);
        p += 64;
        len -= 64;
    }

    if (len) {
        /* Buffer Bounds: left is 0 or left + len < 64 */
        memcpy(ctx->buffer + left, p, len);
    }
    return 0;
}

static inline int SHA256_Final(uint8_t hash[32], SHA256_CTX *ctx) {
    if (!ctx || !hash) return -1;

    static const uint8_t padding[64] = {
        0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    uint64_t total_bits = (uint64_t)ctx->total_len << 3;
    uint32_t left = (uint32_t)(ctx->total_len & 0x3F);
    uint32_t pad_len = (left < 56) ? (56 - left) : (120 - left);

    SHA256_Update(ctx, padding, pad_len);
    
    uint64_t bits_be = BE64(total_bits);
    SHA256_Update(ctx, &bits_be, 8);

    /* Final state translation to hash buffer */
    for (int i = 0; i < 8; i++) {
        uint32_t val = BE32(ctx->state[i]);
        memcpy(hash + i * 4, &val, 4);
    }

    /* Secure Cleanup: Wipe context from memory before returning */
    memset(ctx, 0, sizeof(SHA256_CTX));
    
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* NANOSHA256_H */
