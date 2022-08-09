#ifndef SIMULATOR_HOOKS_H_
#define SIMULATOR_HOOKS_H_

/*
 * Signal handler for Ctrl_C to cause the program to exit, and generate the
 * profiling info.
 */
void handle_sigint( int signal );

void lvgl_log_callback(const char* msg);

#endif  // SIMULATOR_HOOKS_H_