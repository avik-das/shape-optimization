set size 0.75, 0.75

set term epslatex color
set output "lim-twisted-circle.tex"

set xrange [0:204]

set style line 1 linetype 1 linewidth 4 linecolor rgb "blue"
set style line 2 linetype 1 linewidth 4 linecolor rgb "green"
set style line 3 linetype 1 linewidth 4 linecolor rgb "red"

plot "lim-twisted-circle.dat" using 1:2 \
                              title "Bending Energy" \
                              linestyle 1 \
                              with lines, \
     "lim-twisted-circle.dat" using 1:3 \
                              title "Twist Energy" \
                              linestyle 2 \
                              with lines, \
     "lim-twisted-circle.dat" using 1:4 \
                              title "Spring Energy" \
                              linestyle 3 \
                              with lines
