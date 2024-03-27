## Installation and Configuration of qpOASES

Download/clone the qpOASES repo to your home directory or any directory you preferred

```
git clone https://github.com/coin-or/qpOASES.git
```

We suggest compile qpOASES using CMake

```
cd qpOASES
mkdir build && cmake ..
make -j
sudo make install
```

`sudo make install` will install the qpOASES library to `/usr/local/lib` and the header files to `/usr/local/include`