#!/bin/sh

codeName="route"
exeFile="./$codeName"

if [ $# -lt 1 ]
then
    echo "Please use the following command"
    echo "$0 [name]... " 
    echo "name is which testcase (eg, a1 n2 b3...) want to run, it can be a list separated by space."
    exit
fi

# output file
mkdir $codeName.datas

for foo in $*; do
    echo "=================================================="
    echo "       Running for Benchmark testcase " $foo 
    echo "=================================================="
    if [ $foo == "a1" ]
    then
        caseName=a1
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=25 --p2-box-expand-size=1 --overflow-threshold=200 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "a2" ]
    then
        caseName=a2
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=8  --p2-box-expand-size=1 --overflow-threshold=100 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "a3" ]
    then
        caseName=a3
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=10 --p2-box-expand-size=1 --overflow-threshold=200 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "a4" ]
    then
        caseName=a4
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=15 --p2-box-expand-size=1 --overflow-threshold=250 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "a5" ]
    then
        caseName=a5
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=25 --p2-box-expand-size=1 --overflow-threshold=400 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "b1" ]
    then
        caseName=b1
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=10 --p2-box-expand-size=1 --overflow-threshold=340 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "b2" ]
    then
        caseName=b2
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=10 --p2-box-expand-size=1 --overflow-threshold=5   --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=1 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "b3" ]
    then
        caseName=b3
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=10 --p2-box-expand-size=1 --overflow-threshold=36  --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "b4" ]
    then
        caseName=b4
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=12 --p2-box-expand-size=1 --overflow-threshold=95  --p3-max-iteration=4  --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=1 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n1" ]
    then
        caseName=n1
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=12 --p2-box-expand-size=1 --overflow-threshold=4   --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=1 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n2" ]
    then
        caseName=n2
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=3  --p2-box-expand-size=2 --overflow-threshold=250 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n3" ]
    then
        caseName=n3
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=20  --p2-init-box-size=10 --p2-box-expand-size=1 --overflow-threshold=20  --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15  --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n4" ]
    then
        caseName=n4
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=15 --p2-box-expand-size=1 --overflow-threshold=95  --p3-max-iteration=13 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=1 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n5" ]
    then
        caseName=n5
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=12 --p2-box-expand-size=1 --overflow-threshold=100 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n6" ]
    then
        caseName=n6
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=10 --p2-box-expand-size=1 --overflow-threshold=250 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    elif [ $foo == "n7" ]
    then
        caseName=n7
        $exeFile -i $caseName -o $caseName.$codeName.res --p2-max-iteration=150 --p2-init-box-size=12 --p2-box-expand-size=1 --overflow-threshold=150 --p3-max-iteration=8  --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=1 | tee $caseName.$codeName.log; gzip $caseName.$codeName.res; mv $caseName.$codeName.* $codeName.datas
    fi
done
