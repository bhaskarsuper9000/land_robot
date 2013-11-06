#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

int main ()
{
  int iSecret, iGuess, count = 0;

  /* initialize random seed: */
  srand (time(NULL));

  /* generate secret number between 1 and 10: */
  iSecret = rand() % 10 + 1;


  do {
    count++;
    iSecret = rand();
    printf("%d ", iSecret);

  } while (count < 10);

  puts ("Congratulations!");
  return 0;
}
