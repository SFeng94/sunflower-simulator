#include<stdio.h>
#include<stdlib.h>
#include<math.h>

float my_sqrt(float x);
//float my_sqrt(float x){return sqrtf(x);}

float calc_pi()
{
    float P = 4.0f;
    float p = my_sqrt(8.f);
    float diff = 1;

    while (diff > 0.0000001f)
    {
        float new_P = (2 * p * P) / (p + P);
        float new_p = my_sqrt(p * new_P);
	diff = new_p - p;
        p = new_p;
        P = new_P;
    }

    return p;
}
int main() {
    float v=calc_pi();
//	printf("%g",v);
    return 0;
}

