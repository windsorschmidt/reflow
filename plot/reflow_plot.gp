# colors for input data
set style line 1 lc rgb '#f92672' lt 1 lw 1
set style line 2 lc rgb '#00a8c8' lt 1 lw 1
set style line 3 lc rgb '#d88200' lt 1 lw 1
set style line 4 lc rgb '#75af00' lt 1 lw 1
set style line 5 lc rgb '#9257FF' lt 1 lw 1

# basic plot settings
set title "Reflow Oven"
set xlabel "Time (s)"
set ylabel "Temp (C)"
set datafile sep ','
set tics nomirror out scale 0.75
set key top right
xres = 800
yres = 400

# plot to dummy terminal to get min & max values of x
set terminal unknown
plot "reflow_temp.csv" using ($1/1000):($3/100)
xmin = GPVAL_DATA_X_MIN
xmax = GPVAL_DATA_X_MAX
set xrange [0:xmax-xmin]

# draw a horizontal line for Sn63 Pb37 melting point @ 183C
set arrow from 0,183 to xmax-xmin,183 nohead lt 0 lc rgb '#86C30D'
set label "Sn63 Pb37 liquidus" at 0,183 tc rgb '#86C30D' offset 1,0.5

x0=NaN
y0=NaN

### plot on white background ###

set style line 8 lc rgb '#dddddd' lt 0 lw 1
set style line 9 lc rgb '#444444' lt 1 lw 1
set xtics textcolor rgb "#444444"
set ytics textcolor rgb "#444444"
set xlabel textcolor rgb "#444444"
set ylabel textcolor rgb "#444444"
set key textcolor rgb "#444444"
set title textcolor rgb "#444444"
set grid back ls 8
set border 3 front ls 9

set terminal png size xres,yres \
font 'PF Tempesta Seven,6' background rgb '#ffffff'
set output "reflow_light.png"

plot "reflow_triac2.csv" using ($1/1000-xmin):3 \
     title 'Bottom Output (%)' with lines linestyle 4, \
     "reflow_triac1.csv" using ($1/1000-xmin):3 \
     title 'Top Output (%)' with lines linestyle 3, \
     "reflow_target.csv" using ($1/1000-xmin):3 \
     title 'Profile Target' with lines linestyle 2, \
     "reflow_temp.csv" using ($1/1000-xmin):($3/100) \
     title 'Thermocouple' with lines linestyle 1, \
     "reflow_temp.csv" using \
     (dx=$1-x0,x0=$1,$1/1000-xmin):(dy=$3-y0,y0=$3,100*dy/dx) \
     title 'TC Slope × 10' with lines linestyle 5

### replot on black background ###

set style line 8 lc rgb '#333333' lt 0 lw 1
set style line 9 lc rgb '#bbbbbb' lt 1 lw 1
set xtics textcolor rgb "#bbbbbb"
set ytics textcolor rgb "#bbbbbb"
set xlabel textcolor rgb "#bbbbbb"
set ylabel textcolor rgb "#bbbbbb"
set key textcolor rgb "#bbbbbb"
set title textcolor rgb "#bbbbbb"
set grid back ls 8
set border 3 front ls 9

set terminal png size xres,yres \
font 'PF Tempesta Seven,6' background rgb '#000000'
set output "reflow_dark.png"
replot
