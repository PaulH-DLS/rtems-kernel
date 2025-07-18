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

#include <stdlib.h>

int main(void)
{
  void *a;
  int ret = posix_memalign(&a, sizeof (void *) * 2, 42);

  return ret;
}
