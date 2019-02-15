# About

The included script can be used to plot data stored in a file, as sent by the controller. Data are stored line-wise as JSON strings.

## Requirements:

- gnuplot - http://www.gnuplot.info/
- jq - https://stedolan.github.io/jq/

## Plotting

Run the script, passing the name of a log file as the argument (an example file is included):

```
$ sh make_plot.sh reflow_example.log
```

You may get a warning from gnuplot about plotting with an 'unknown' terminal -- this can be ignored. The resulting plots (with light and dark background) are generated in the current directory.

![Plot of data logged by the controller](example_plot.png?raw=true "Plot of data logged by the controller")
