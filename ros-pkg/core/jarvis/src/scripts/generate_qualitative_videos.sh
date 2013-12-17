#!/bin/bash

# -- Parse args.
if [ "$#" == "0" ]; then
    echo "Usage: $0 CLASSIFIER CLASSNAME TD [ TD ... ]"
    echo "  Generates CLASSNAME-{pos,neg}.avi in the current working directory."
    echo "  If you kill this midway through, a potentially large temp directory will be left behind."
    exit 1
fi

CLASSIFIER=$1
CLASSNAME=$2
shift 2
TDS=$@

echo $CLASSIFIER
echo $CLASSNAME
echo $TDS

TMPDIR=`mktemp -d -u generate_qualitative_videos-XXXXXXXXX`
echo
echo $TMPDIR
mkdir $TMPDIR

echo
echo == Copying TDs.
rsync -av $TDS $TMPDIR/ --progress

echo
echo == Classifying.
rosrun online_learning classify $CLASSIFIER --tds $TMPDIR/*.td

echo
echo == Filtering.
mkdir $TMPDIR/filtered
rosrun online_learning filter_tracks --pos $CLASSNAME --tds $TMPDIR/*.td -o $TMPDIR/filtered/${CLASSNAME}-pos.td
rosrun online_learning filter_tracks --neg $CLASSNAME --tds $TMPDIR/*.td -o $TMPDIR/filtered/${CLASSNAME}-neg.td

echo
echo == Generating videos.
for idx in `seq -f'%02g' 0 5`; do 
    rosrun jarvis generate_collage --tds $TMPDIR/filtered/${CLASSNAME}-pos.td -s 5 -v
    rosrun jarvis generate_collage --tds $TMPDIR/filtered/${CLASSNAME}-neg.td -s 5 -v
    mv $TMPDIR/filtered/${CLASSNAME}-pos.avi ${CLASSNAME}-pos-${idx}.avi
    mv $TMPDIR/filtered/${CLASSNAME}-neg.avi ${CLASSNAME}-neg-${idx}.avi
done

rm -rf $TMPDIR
    