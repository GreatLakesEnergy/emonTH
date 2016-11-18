#!/bin/bash
#python WirelessProgramming.py -f node_analogRead_blink.hex -t 5 -s /dev/ttyO2 2>&1 | grep "SUCCESS!"

function myFunction() {
	#this is for one node
	python WirelessProgramming.py -f node_analogRead_blink.hex -t 5 -s /dev/ttyO2
	return $?
}

retry=0
maxRetries=10
retryInterval=10
until [ ${retry} -ge ${maxRetries} ]
do
	#myFunction && break
	echo $?
	myFunction && break
	retry=$[${retry}+1]
	echo "Retrying [${retry}/${maxRetries}] in ${retryInterval}(s) "
	sleep ${retryInterval}
done

if [ ${retry} -ge ${maxRetries} ]; then
  echo "Failed after ${maxRetries} attempts!"
  exit 1
fi

#if [[ ! $? -eq 1 && ! $init ]]; then
#        echo 'WSN programmed successful!!!!!!!!'
#else
#        echo 'shit happens ##################'
#        while [[ ! $? -eq 1 && ! $init ]]; do
#                python WirelessProgramming.py -f node_analogRead_blink.hex -t 5 6 4 -s /dev/ttyO2
#        done

#fi

