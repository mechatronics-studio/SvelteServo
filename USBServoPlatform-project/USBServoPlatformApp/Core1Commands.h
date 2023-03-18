#ifndef CORE1_COMMANDS
#define CORE1_COMMANDS


/**
 * If there is command on the core1 command queue, it is executed.
*/
void pop_and_execute_core1_command();

void setup_core1_command_chain();

#endif