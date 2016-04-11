clc
clear all
close all

k1=1;
k2=1;
b1=1;
b2=1;
j1=1;
j2=1;

num1=[j2+b2 (k1+k2)];
den1=[(j1*j2) (b2*j1+b1*j2) (k1+j2*k2+b1*b2+k2) (b1*k1+b1*k2+b2*k2) (k1*k2)];

num2=[1];
den2=[((j1*j2)/k2) ((1/k2)*(j2*b1+b2*j1)) (j2+((b1*b2)/k2)) (((j1*k1)/k2)+j1) (b2+((b1*k1)/k2)+b1) (1+k1-k2)];