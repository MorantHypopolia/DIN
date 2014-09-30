#!/bin/bash

i=0

echo "Press 'f' to move forward or press 'b' to move backward"

while true; do
	echo "Iteration $i"
	`gnuplot -p -e "plot './iteration_$i' u 1:2 w p notitle, './iteration_$i' u 3:4 w p pt 6 ps 4 lt rgb 'black' notitle"`
	wmctrl -a "Analyzer"
	read -n 1 key
	
	if [ $key == "f" ]; then
		let i=$i+1
	elif [ $key == "b" ]; then
		let i=$i-1
		
		if [ $i -lt 0 ]; then
			let i=0
		fi
	fi
	
	killall gnuplot
done
