#!/bin/bash

. ~/.bashrc  # Set up ROS.

# -- Parse args.
if [ "$#" == "0" ]; then
    echo "Usage: $0 TEST_DIR CONFIG RUN_BASE_DIR"
    echo "  TEST_DIR is the location of all setup data needed for a test."
    echo "  RUN_BASE_DIR is where output will be saved.  The run dir will be created within this."
    exit 1
fi

TEST_DIR=$1
CONFIG=$2
RUN_BASE_DIR=$3
CLASS_NAMES=`cat $TEST_DIR/class_names.txt`
MAX_ITERS=`cat $TEST_DIR/max_iters.txt`

echo -e "Test dir:\t" $TEST_DIR
echo -e "Config file:\t" $CONFIG
echo -e "Max iters:\t" $MAX_ITERS
echo -e "Run base dir:\t" $RUN_BASE_DIR
echo -e "Class names:\t" $CLASS_NAMES

TEST_NAME=`basename $TEST_DIR`
echo -e "Test name:\t" $TEST_NAME

RUN_DIR=$RUN_BASE_DIR/`date +%F_%T`_`roscd jarvis && git rev-parse --short HEAD`_$TEST_NAME
echo -e "Run dir:\t" $RUN_DIR

mkdir -p $RUN_DIR/induction
rosrun jarvis induct \
    --randomize \
    --no-vis \
    --max-iters $MAX_ITERS \
    --saved-annotations-dir $TEST_DIR/saved_annotations \
    --class-names $CLASS_NAMES \
    --config $CONFIG \
    -u $TEST_DIR/up.eig.txt \
    --emax 0 \
    --buffer-size 1000 \
    --max-track-length 30 \
    --snapshot-every 0 \
    --evaluate-every 1 \
    --output-dir $RUN_DIR/induction \
    --unlabeled-td-dir $TEST_DIR/unlabeled \
    --init $TEST_DIR/init/*.td \
    --test $TEST_DIR/test/*.td \
    --fake-supervisor $TEST_DIR/supervisor.gc \
    | tee $RUN_DIR/induction/log.txt


# -- Plot.
cd $RUN_DIR/induction
for name in $CLASS_NAMES; do
    rosrun online_learning plot_perclass_pr.py $name
    rosrun online_learning plot_perclass_accuracy.py $name
done
cd -

# -- Run the baseline.
echo
echo -- Running baseline_unfair --
echo

mkdir -p $RUN_DIR/baseline_unfair
rosrun jarvis baseline_unfair \
    --randomize \
    --class-names $CLASS_NAMES \
    --config $CONFIG \
    -u $TEST_DIR/up.eig.txt \
    --root $RUN_DIR/induction \
    --test $TEST_DIR/test/*.td \
    --num-iters 10 \
    --o $RUN_DIR/baseline_unfair \
    | tee $RUN_DIR/baseline_unfair/log.txt

# -- Delete things we don't need to save disk space.
echo
echo -- Cleaning up. --
echo
cp `find $RUN_DIR/induction -name classifier.gc | sort | tail -n1` $RUN_DIR/induction/final_classifier.gc
cp `find $RUN_DIR/induction -name track_results.txt | sort | tail -n1` $RUN_DIR/induction/final_track_results.txt
rm `find $RUN_DIR/induction -name classifier.gc`
rm `find $RUN_DIR/induction -name 'annotated*.td'`
rm `find $RUN_DIR/induction -name '*.eig'`
rm `find $RUN_DIR/induction -name cmap.txt`
rm -rf `find $RUN_DIR/induction -name test_results_annotated`