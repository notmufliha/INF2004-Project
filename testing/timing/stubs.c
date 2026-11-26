/* stubs.c - kernel/console calls so a module links on its own for the
 * cycle counter.  disint/enaint do what the armv6m kernel versions do
 * (read PRIMASK, CPSID / restore), so interrupt locking is counted. */
typedef unsigned int UW;
UW disint(void)
{
	UW s;
	__asm volatile ("mrs %0, primask\n cpsid i" : "=r"(s) :: "memory");
	return s;
}
void enaint(UW s) { __asm volatile ("msr primask, %0" :: "r"(s) : "memory"); }
int tm_printf(const char *f, ...) { (void)f; return 0; }
void fault_raise(unsigned f) { (void)f; }
void _exit(int c) { (void)c; for (;;) { } }
