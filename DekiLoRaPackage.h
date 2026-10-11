#pragma once

#ifdef _WIN32
#ifdef DEKI_LORA_EXPORTS
#define DEKI_LORA_API __declspec(dllexport)
#else
#define DEKI_LORA_API __declspec(dllimport)
#endif
#else
#define DEKI_LORA_API __attribute__((visibility("default")))
#endif
