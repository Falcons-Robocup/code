#!/bin/bash
#


if [ ! -z "$1" ]; then
   if [ "$1" = "clear" ]; then
      if [ -f /var/tmp/tracing_trigger ]; then
         rm /var/tmp/tracing_trigger
      else
         exit 0
      fi
   else
      echo "$1" > /var/tmp/tracing_trigger
   fi
else
   touch /var/tmp/tracing_trigger
fi

