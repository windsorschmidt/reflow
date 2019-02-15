#!/bin/sh

if [ $# -eq 0 ]; then
    echo "Usage: $0 <infile>"
    exit 1
fi

# reflow profile target temperature
cat $1 | \
    jq -r -c "select(.cmd==1537)|[.t,.cmd,.arg]" | \
    tr -d '[]' > reflow_target.csv

# thermocouple probe temperature
cat $1 | \
    jq -r -c "select(.cmd==257)|[.t,.cmd,.arg]" | \
    tr -d '[]' > reflow_temp.csv

# triac 1 duty cycle
cat $1 | \
    jq -r -c "select(.cmd==769)|[.t,.cmd,.arg]" | \
    tr -d '[]' > reflow_triac1.csv

# triac 2 duty cycle
cat $1 | \
    jq -r -c "select(.cmd==770)|[.t,.cmd,.arg]" | \
    tr -d '[]' > reflow_triac2.csv

gnuplot reflow_plot.gp
