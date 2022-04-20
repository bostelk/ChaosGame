export void simple(uniform float vin[], uniform float vout[],
                   uniform int count) {
    foreach (index = 0 ... count) {
        // Load the appropriate input value for this program instance.
        float v = vin[index];

        // Do an arbitrary little computation, but at least make the
        // computation dependent on the value being processed
        if (v < 3.)
            v = v * v;
        else
            v = sqrt(v);

        // And write the result to the output array.
        vout[index] = v;
    }
}

export void sierpinski(uniform float xin[], uniform float yin[], uniform int selectorin[], uniform float xout[], uniform float yout[],
                   uniform int iterations,
                   uniform int count) {
    foreach (index = 0 ... count) {
        // Load the appropriate input value for this program instance.
        float x = xin[index];
        float y = yin[index];

        int* selectorin_off = selectorin + index * 20;

        for (int i = 0; i < iterations; ++i) {
            int selector = selectorin_off[i];
            if (selector == 0)
            {
                x = x / 2;
                y = y / 2;
            }
            else if (selector == 1)
            {
                x = (x + 1) / 2;
                y = y / 2;
            }
            else if (selector == 2)
            {
                x = x / 2;
                y = (y + 1) / 2;
            }
        }

        // And write the result to the output array.
        xout[index] = x;
        yout[index] = y;
    }
}

export void affine(uniform float xin[], uniform float yin[], uniform int selectorin[], uniform float transformin[], uniform float xout[], uniform float yout[],
                   uniform int iterations,
                   uniform int count) {
    foreach (index = 0 ... count) {
        // Load the appropriate input value for this program instance.
        float x = xin[index];
        float y = yin[index];

        int* selectorin_off = selectorin + index * iterations;

        for (int i = 0; i < iterations; ++i) {
            float* transformin_off = transformin;

            int selector = selectorin_off[i];
            if (selector == 1)
            {
                // Next matrix.
                transformin_off += 6;
            }

            float x1 = transformin_off[0] * x + transformin_off[2] * y + transformin_off[4];
            float y1 = transformin_off[1] * x + transformin_off[3] * y + transformin_off[5];

            x = x1;
            y = y1;

            // Spherical func.
            float r = sqrt(x*x + y*y);
            float r2 = r * r;

            x = x / r2;
            y = y / r2;
        }

        // And write the result to the output array.
        xout[index] = x;
        yout[index] = y;
    }
}