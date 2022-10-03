# Getting Started

UBC Sailbot's Network Systems team uses C++ for its software. If you know already know C, then you already know the
bare minimum to write C++. This is a good starting point, but the additional features C++ provides allow for safer
programming practices.

## For C/C++ Beginners

If you just need to know how C++ is different from C, then see the [Differences Between C and C++](#differences-between-c-and-c-plus-plus).
You should also look at it if you go through and finish this section.

If you are new to C and C++, then this the best place to start. The tutorials provided in this section will help you
learn the fundamentals of the language. Do not feel pressured to do all the tutorials! Just get comfortable with the
syntax and the mechanisms of the language.

!!! note
    The hardest part about this will likely be pointers and dynamic memory, so
    pay close attention to tutorials concerning them!

| Resource               | Description                                                                                 |
| :--------------------- | :------------------------------------------------------------------------------------------ |
| [w3schools Tutorial](https://www.w3schools.com/cpp/default.asp) | A structured tutorial that goes through basic concepts in C++. It's good to do up to the section on Classes. |
| [YouTube Tutorial](https://youtu.be/vLnPwxZdW4Y) | If you prefer video tutorial, then this is a comprehensive 4 hour video covering similar concepts to the one above. It is 4 hours long though. |
| [Dynamic Memory Overview](https://www.tutorialspoint.com/cplusplus/cpp_dynamic_memory.htm) | A page going over how dynamic memory works in C++. |

Feel free to add other resources other than the ones listed above if you find any that you like!

## Differences Between C and C Plus Plus

For most use cases, you can think of C++ as a superset of C. While this is not technically true, more often than not
you are able to write standard C code for a C++ program without issues. However, doing so ignores a lot of the benefits
and reasons to use C++.

### Classes and Structs

In C structs can only contain member variables, but in C++ structs are basically classes but with a default member
visibility of public instead of private.

!!! example
    The following code blocks are equivalent.

    ```C++
    struct foo {
    private:
        int x;
        void helper(void);
    public:
        foo(int y);
    }
    ```

    ```C++
    class foo {
    private:
        int x;
        void helper(void);
    public:
        foo(int y);
    }
    ```

### Namespaces

One problem that is prevalent in C concerns the scoping of names. For example, let there be two files `A.h` and `B.h`
and a program `ighxy.c` wants to use both of them.

```C
// A.h
float x;
int bar(void);

// B.h
float x;
int bar(void);

// ighxy.c
#include "A.h"
#include "B.h"

int main(void) {
    int a = bar();
    ...
}
/* Error, does not compile*/
```

Our program cannot compile because the linker cannot distinguish which `bar()` function we want to use! One way to fix
this in a C program would be to rename them `a_bar()` and `b_bar()`. Although this fix seems trivial for this example,
applying it to a file that has potentially 100 functions can be a lot more difficult, especially if two files just
happen to share the same prefix for their functions!

C++ introduces namespaces.

```C++
// A.h
namespace a {
float x;
int bar(void);
}  // namespace a

// B.h
namespace b {
int bar(void);
}  // namespace b

// ighxy.cpp
#include "A.h"
#include "B.h"

int main(void) {
    int a = a::bar();
    int b = b::bar();
    float xa = a::x;
    float xb = b::x;
    ...
}
```

With namespaces, we can deal with naming conflicts much more easily. Though be aware that namespaces are not necessary
everywhere.

### Constant Expressions

In C, if we want to declare a constant or a function/expression that we want to be evaluated at compile time, we need
to use `#define` statements. One of the problems with `#define` statements is that they perform a simple copy paste
wherever they're used. For example:

```C
#define PI 3.14F
#define AREA_OF_CIRCLE(radius) ((PI) * (radius) * (radius))

int main(void) {
    // Before precompile
    float area = AREA_OF_CIRCLE(2.5F);
    // After precompile
    float area = ((3.14F) * (2.5F) * (2.5F));
    ...
}
```

Because of this copy-pasting, you need to be very careful with syntax, sometimes necessitating an ugly [`do {} while(0)`
wrapper](https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block). Moreover, symbols
declared with `#define` are always globally visible, even ignoring namespaces!

In C++, the use of constant expressions are preferred.

```C++
constexpr float pi = 3.14F;
constexpr float area_of_circle(float radius) {
    return pi * radius * radius;
}
```

Constant expressions do *not* get copy pasted, and are instead placed in program memory just like a normal variable
or function. They also respect namespaces and function scopes, meaning the following code compiles.

```C++
void foo(void) {
    constexpr float rand = 123.456;
}

void bar (void) {
    constexpr float rand = 789.123;
}
```

### Lambdas

Lambdas are primarily useful when you need to register a callback function one time and don't feel it's necessary to
write out a full function. They are in no way required though, so do not worry about learning them. However, it's
necessary to know that they exist such that you don't get confused when reading code. For more information, [go here](https://learn.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170)
for Microsoft's explanation.
