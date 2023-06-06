# LTC3337 Unit Test

## Summary
This unit test provides verification of the 32-bit integer based math used in 
the LTC3337 driver.  The baseline/expected values for the unit test are 
generated using double precision floating point math, coded independently from
the datasheet specifications

## Requirements
The unit tests can be run on any development platform, such as Linux, Cygwin, or
MinGW provided by the Maxim MSDK. The unit test utilizes the [cppUTest Framework](http://cpputest.github.io/),
which can be installed on the development platfrom following the cppuTest
instructions.  

## Building
To build the unit test, simply run make on the development platform.  The 
provided makefile references /mingw64 as the library and include path for 
cppUTest, which is the default install path when building on MinGW.  Modify 
these paths if running on a different platform.

## Running
To run the unit tests, simply run the built ltc3337_UnitTest application. This
will execute all defined tests and provide the results

```
$ ./ltc3337_UnitTest.exe
.....
OK (5 tests, 5 ran, 16781769 checks, 0 ignored, 0 filtered out, 505 ms)
```