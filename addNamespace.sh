#!/bin/sh

NAMESPACE=NTHUR
for x in $(find . -name "*.h"); do
    sed -i "$(grep -n "^#" $x | tail -2 | head -1 | sed 's/:.*//')a\\\nnamespace $NAMESPACE {\n" $x
    sed -i "$(($(grep -n "^#" $x | tail -1 | sed 's/:.*//')-1))a} // namespace $NAMESPACE\n" $x
done

for x in $(find . -name "*.cpp"); do
    sed -i "$(grep -n "#include" $x | tail -1 | sed 's/:.*//')a\\\nnamespace $NAMESPACE {\n" $x
    echo >> $x
    echo "} // namespace $NAMESPACE" >> $x
done
