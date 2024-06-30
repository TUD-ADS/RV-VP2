int main() {
	volatile int a = 1 | 2 << 8 | 3 << 16 | 4 << 24;
	volatile int b = 4 | 3 << 8 | 2 << 16 | 1 << 24;
	asm("ADD8 s11, %0, %1" : : "r"(a), "r"(b));

	return 0;
}
