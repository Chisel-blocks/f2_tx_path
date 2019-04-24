#!/bin/sh
#Init submodules in this dir, if any
DIR="$( cd "$( dirname $0 )" && pwd )"
git submodule update --init

#!/bin/sh
#Init submodules in this dir, if any
DIR="$( cd "$( dirname $0 )" && pwd )"
git submodule update --init

##Publish local the ones you need
cd $DIR/rocket-chip
git submodule update --init firrtl
git submodule update --init chisel3
git submodule update --init hardfloat

cd $DIR/rocket-chip/firrtl
sbt publishLocal
cd $DIR/rocket-chip/chisel3
sbt publishLocal

cd $DIR/rocket-chip
sbt publishLocal

SUBMODULES="\
    f2_interpolator \
    clkmux \
    prog_delay \
    " 
for module in $SUBMODULES; do
    cd ${DIR}/${module}
    if [ -f "./init_submodules.sh" ]; then
        ./init_submodules.sh
    fi
    sbt publishLocal
done

exit 0

