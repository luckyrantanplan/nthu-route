#!/bin/sh

project="nthuRouter3"

mkdir -p /home/florian/Documents/${project}/src
cp ~/eclipse-workspace/ISPD2008-NTHU-R-CodeRelease-Updated/src/* /home/florian/Documents/${project}/src -r
cp ~/eclipse-workspace/ISPD2008-NTHU-R-CodeRelease-Updated/Debug/* /home/florian/Documents/${project} -r
cd /home/florian/Documents/${project}
sed -i s/"..\/src\/"/".\/src\/"/g  `fgrep "../src" -rl`
make clean
cd /home/florian/Documents
tar -czf nthuRoute3.tar.gz --directory=/home/florian/Documents/ ${project}  
rm /home/florian/Documents/${project} -rf
