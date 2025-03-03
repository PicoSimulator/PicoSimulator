
#include <stdint.h>
#include "pico/printf.h"
#include "pico/stdio.h"
#include "sha256.h"
#include <string.h>
#include "hardware/structs/systick.h"

typedef struct{
  uint8_t digest[32];
} hash_t;

typedef hash_t test_return_t;

void print_hash(hash_t *hash)
{
  for (int i = 0; i < 32; i++)
    printf("%02x ", hash->digest[i]);
}

hash_t create_named_digest_u32(const char* desc, uint32_t val)
{
  hash_t hash;
  SHA256_CTX ctx;
  sha256_init(&ctx);
  sha256_update(&ctx, (const uint8_t*)desc, strlen(desc));
  sha256_update(&ctx, (const uint8_t*)&val, sizeof(val));
  sha256_final(&ctx, hash.digest);
  print_hash(&hash);
  printf(" create_digest_u32(\"%s\", %u)\n", desc, val);
  return hash;
}

void digest_combine(hash_t *hash, hash_t *other)
{
  SHA256_CTX ctx;
  sha256_init(&ctx);
  sha256_update(&ctx, hash->digest, 32);
  sha256_update(&ctx, other->digest, 32);
  sha256_final(&ctx, hash->digest);
}

test_return_t test_instruction_timing()
{
  uint32_t tickcnt = systick_hw->cvr;
  return create_named_digest_u32(__PRETTY_FUNCTION__, tickcnt);
}

int main(){

  stdio_init_all();

  test_return_t hash = test_instruction_timing();

  while(true){
    asm("wfi");
  }

  return 0;
}