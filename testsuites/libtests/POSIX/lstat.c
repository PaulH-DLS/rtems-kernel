/*
 * Copyright (c) 2009 by
 * Ralf Corsepius, Ulm, Germany. All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software
 * is freely granted, provided that this notice is preserved.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

int main(void)
{
  struct stat buf;
  int status;

  status = lstat("/tmp/foo", &buf);

  return status;
}
