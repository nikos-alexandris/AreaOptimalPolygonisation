#include "Common.hpp"

namespace project {

[[noreturn]] void unreachable() { __builtin_unreachable(); }

NotImplementedError::NotImplementedError() : std::logic_error("Function not yet implemented") {}

// https://stackoverflow.com/a/44910009
char* sprintf_int128(__int128 n) {
	static char str[41] = {0};		 // sign + log10(2**128) + '\0'
	char* s = str + sizeof(str) - 1; // start at the end
	bool neg = n < 0;
	if (neg)
		n = -n;
	do {
		*--s = "0123456789"[n % 10]; // save last digit
		n /= 10;					 // drop it
	} while (n);
	if (neg)
		*--s = '-';
	return s;
}

} // namespace project