br3ttb's PID library, ported to Kotlin
======================================

This project contains a port of the [Arduino PID](https://github.com/br3ttb/Arduino-PID-Library)
library and its [tester](https://github.com/br3ttb/arduino-pid-library-tester),
both made by br3ttb. They are in the `lib` and the `tester` modules
respectively.
 
For any information about PID control in general, have a look at the links above
or [Wikipedia](https://en.wikipedia.org/wiki/PID_controller).


The library and its tester differ from the original in these aspects:
* I added a `timeFunction` callback to make it easier to test by simulate
time and eliminate the effects of non-real-time behavior in x86 architecture.
* the original code uses pointers to
variables to read/write the input, output and setpoint. Kotlin doesn't
support pointers (not on JVM at least), so here I use callbacks and lambdas.

## TODO
* Ensure that this library outputs exactly the same as the original library.

