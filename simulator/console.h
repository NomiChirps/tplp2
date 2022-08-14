#ifndef CONSOLE_H
#define CONSOLE_H

// Thread-safe printf wrapper.

void console_init(void);
void console_print(const char* fmt, ...);

#endif /* CONSOLE_H */