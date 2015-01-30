#!/bin/bash
if [ `getconf LONG_BIT` = "64" ]; then
  # 64-bit stuff here
  tar xvf libtcod-1.5.1-linux64.tar.gz
else
  # 32-bit stuff here
  tar xvf libtcod-1.5.1-linux.tar.gz
fi
cd libtcod-1.5.1/
mv include/ libtcod-1.5.1
sudo cp -r libtcod-1.5.1/ /usr/local/include/
sudo cp ./{libtcod_debug.so,libtcodgui_debug.so,libtcodgui.so,libtcod.so,libtcodxx_debug.so,libtcodxx.so} /usr/local/lib/
cd ..
rm -rf libtcod-1.5.1/
echo "DONE!"  

