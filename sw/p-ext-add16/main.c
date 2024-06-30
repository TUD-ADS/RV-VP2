int main() {
	volatile int a = 1 | 2 << 16;
	volatile int b = 4 | 2 << 16;
	asm("ADD16 s11, %0, %1" : : "r"(a), "r"(b));

	return 0;
}
