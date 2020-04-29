# source
# shellcheck source=src/lib.sh
#source ~/.bashrc

# roscd benchmark

# load json parameters
if [ "$(jq '.rviz' settings/settings.json)" == false ] ; then
  export ENABLE_RVIZ=False
else
  export ENABLE_RVIZ=True
fi
if [ "$(jq '.rqt' settings/settings.json)" == false ] ; then
  export ENABLE_RQT=False
else
  export ENABLE_RQT=True
fi
NUMBER_OF_ROBOTS=$(jq '.number_of_robots' settings/settings.json)
NAMESPACE=$(jq '.namespace' settings/settings.json)
MODEL_NAME=$(jq '.model_name' settings/settings.json)
MODEL_TYPE=$(jq '.model_type' settings/settings.json)
export NUMBER_OF_ROBOTS
export NAMESPACE
export MODEL_NAME
export  MODEL_TYPE

# load via python
