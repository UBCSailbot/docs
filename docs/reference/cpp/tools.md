# Tools
A lot goes into making a well structured C++ project, much more than any one team should have to do.

## CMake
CMake is a powerfull build automization tool that makes compiling code for large projects with a lot of interoperating
files a lot easier. Steps 1-3 of the [official tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
are great for understanding the basics.

## GoogleTest
[GoogleTest](https://github.com/google/googletest) is the C++ unit testing framework we will be using.
The [GoogleTest Primer](https://google.github.io/googletest/primer.html) is a good place to start.

<!-- ## Google Mock Not sure if we're going to use this yet -->

## Google Protocol Buffer
[Google Protocol Buffer](https://developers.google.com/protocol-buffers) (ProtoBuf) is a portable data serialization
method. We use it over other methods like JSON and XML because it produces smaller binaries, an important consideration
when sending data across an ocean. Unfortunately, there does not seem to be a easy to follow tutorial for using them,
but here are the [C++ basics](https://developers.google.com/protocol-buffers/docs/cpptutorial). The page is quite dense
and can be hard to follow, so do not worry if you do not understand it.

## Clang
In its most basic form, [Clang](https://clang.llvm.org/) is a compiler for the C language family. Clang has multiple
benefits like easier portability compared to, for example, GCC. Clang is actually "half" the compiler, the other half
being LLVM. Without going into unnecessary detail, Clang compiles C++ code to a generic language before LLVM compiles
it to machine specific language.
### Clangd
[Clangd](https://clangd.llvm.org/) is the Clang language server. It provides a much more powerful intellisense than the
default one used in VSCode's C/C++ extension.
### Clang-Tidy
[Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/) is a linting tool, who's main purpose is to catch potential
programming errors caused by bad programming style/practices using just static analysis.
### Clang Format
An autoformatting tool that makes enforcing style guidelines much easier. When se tup, it corrects formatting as soon
as you hit save.

## llvm-cov
We will use [llvm-cov](https://llvm.org/docs/CommandGuide/llvm-cov.html) to evaluate our test coverage. When used with
[genhtml](https://www.systutorials.com/docs/linux/man/1-genhtml/), we can generate HTML reports that that show our line,
function, and branch coverage line-by-line.
