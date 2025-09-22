clear all
close all
clc

t = tcpclient("127.0.0.1",10002);

read(t)