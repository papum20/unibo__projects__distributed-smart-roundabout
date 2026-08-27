#!/bin/bash

pip list --format=columns | tail -n +3 | awk '{print $1"==" $2}'