# Example Makefile

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -Werror

# Source files
SRCS = main.c common.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable name
EXEC = my_program

# Default target
all: $(EXEC)

# Link object files to create executable
$(EXEC): $(OBJS)
    $(CC) $(CFLAGS) -o $@ $^

# Compile source files to object files
%.o: %.c
    $(CC) $(CFLAGS) -c $< -o $@

# Clean up generated files
clean:
    rm -f $(OBJS) $(EXEC)