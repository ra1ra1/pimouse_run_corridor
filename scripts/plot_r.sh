#!/bin/sh

# values.csvを描画
gnuplot -e "
  set terminal vttek;
  set datafile separator ',';
  plot [-50:100][-50:100] 'map2.csv'
  "
