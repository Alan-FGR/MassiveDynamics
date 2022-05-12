#pragma once
#undef interface // this is defined in Windows SDK's combaseapi.h
#define interface export struct __declspec(novtable)