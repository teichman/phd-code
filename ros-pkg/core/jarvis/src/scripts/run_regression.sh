#!/bin/bash

. ~/.bashrc  # Set up ROS.

# -- Parse args.
if [ "$#" == "0" ]; then
    echo "Usage: $0 TEST_DIR CONFIG MAX_ITERS RUN_BASE_DIR"
    echo "  TEST_DIR is the location of all setup data needed for a test."
    echo "  RUN_BASE_DIR is where output will be saved.  The run dir will be created within this."
    exit 1
fi

TEST_DIR=$1
CONFIG=$2
MAX_ITERS=$3
RUN_BASE_DIR=$4
CLASS_NAMES=`cat $TEST_DIR/class_names.txt`
#MAX_ITERS=`cat $TEST_DIR/max_iters.txt`

echo -e "Test dir:\t" $TEST_DIR
echo -e "Config file:\t" $CONFIG
echo -e "Max iters:\t" $MAX_ITERS
echo -e "Run base dir:\t" $RUN_BASE_DIR
echo -e "Class names:\t" $CLASS_NAMES

TEST_NAME=`basename $TEST_DIR`
echo -e "Test name:\t" $TEST_NAME

RUN_DIR=$RUN_BASE_DIR/`date +%F_%T`_`roscd jarvis && git rev-parse --short HEAD`_$TEST_NAME
echo -e "Run dir:\t" $RUN_DIR

echo
echo -- Running group induction --
echo

mkdir -p $RUN_DIR/induction
echo `hostname` > $RUN_DIR/hostname.txt
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
    --fake-supervisor-config $TEST_DIR/supervisor_config.yml \
    | tee $RUN_DIR/induction/log.txt

cd $RUN_DIR/induction
for name in $CLASS_NAMES; do
    rosrun online_learning plot_perclass_pr.py $name
    rosrun online_learning plot_perclass_accuracy.py $name
done
cd -

cp `find $RUN_DIR/induction -name classifier.gc | sort | tail -n1` $RUN_DIR/induction/final_classifier.gc
cp `find $RUN_DIR/induction -name track_results.txt | sort | grep test_results | tail -n1` $RUN_DIR/induction/final_track_results.txt

echo
echo -- Running active learning baseline --
echo

mkdir -p $RUN_DIR/active_learning
echo `hostname` > $RUN_DIR/hostname.txt
rosrun jarvis induct \
    --active-learning \
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
    --output-dir $RUN_DIR/active_learning \
    --unlabeled-td-dir $TEST_DIR/unlabeled \
    --init $TEST_DIR/init/*.td \
    --test $TEST_DIR/test/*.td \
    --fake-supervisor $TEST_DIR/supervisor.gc \
    --fake-supervisor-config $TEST_DIR/supervisor_config.yml \
    --fake-supervisor-annotation-limit `grep -A2 'Hand-annotated' $(find $RUN_DIR/induction/ -name learner_status.txt | sort | tail -n1) | grep tracks | awk '{print $1}'` \
    | tee $RUN_DIR/active_learning/log.txt

cd $RUN_DIR/active_learning
for name in $CLASS_NAMES; do
    rosrun online_learning plot_perclass_pr.py $name
    rosrun online_learning plot_perclass_accuracy.py $name
done
cd -

cp `find $RUN_DIR/active_learning -name classifier.gc | sort | tail -n1` $RUN_DIR/active_learning/final_classifier.gc
cp `find $RUN_DIR/active_learning -name track_results.txt | sort | grep test_results | tail -n1` $RUN_DIR/active_learning/final_track_results.txt

echo
echo -- Running naive supervised baseline --
echo

mkdir -p $RUN_DIR/naive_supervised_baseline
rosrun jarvis naive_supervised_baseline \
    --config $CONFIG \
    -u $TEST_DIR/up.eig.txt \
    --class-names $CLASS_NAMES \
    --train $TEST_DIR/nsb_training/*.td \
    --test $TEST_DIR/test/*.td \
    --num-runs 15 \
    --subsample `grep -A2 'Hand-annotated' $(find $RUN_DIR/induction/ -name learner_status.txt | sort | tail -n1) | grep tracks | awk '{print $1}'` \
    -o $RUN_DIR/naive_supervised_baseline \
    -j 24 \
    --randomize \
    | tee $RUN_DIR/naive_supervised_baseline/log.txt

echo
echo -- Running matched supervised baseline --
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
rm `find $RUN_DIR/induction -name classifier.gc | sort | sed '$d'`
rm `find $RUN_DIR/induction -name 'annotated*.td'`
rm `find $RUN_DIR/induction -name '*.eig'`
rm `find $RUN_DIR/induction -name cmap.txt`
rm -rf `find $RUN_DIR/induction -name annotation_results`
rm -rf `find $RUN_DIR/induction -name validation_results`
rm `find $RUN_DIR/active_learning -name classifier.gc | sort | sed '$d'`
rm `find $RUN_DIR/active_learning -name 'annotated*.td'`
rm `find $RUN_DIR/active_learning -name '*.eig'`
rm `find $RUN_DIR/active_learning -name cmap.txt`
rm -rf `find $RUN_DIR/active_learning -name annotation_results`
rm -rf `find $RUN_DIR/active_learning -name validation_results`



