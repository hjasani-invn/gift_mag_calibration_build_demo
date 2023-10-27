#ifndef ALTITUDE_FILTER_HPP
#define ALTITUDE_FILTER_HPP

static double const filter_coef_[] =
{
0.000978577031845619463301666485222085612,
0.001851774076405747005216739786703783466,
0.003036697882958972090350346917375645717,
0.004567208755585410763500053832331104786,
0.006466728494857592983435790046087277005,
0.008745562014262435110434346086094592465,
0.011398705722202287851652080519215815002,
0.014404318807632635404680776503028027946,
0.01772300189523110663936122932682337705 ,
0.021297984345923545723699987775034969673,
0.02505626930118943357372884861433703918 ,
0.028910727678562592180000478947476949543,
0.032763072517720592813361690787132829428,
0.03650758740535271773541836637377855368 ,
0.040035431192303370939722384491687989794,
0.043239299548641835224671581272559706122,
0.046018195165831499160447037866106256843,
0.048282044852593269945728593484091106802,
0.049955904605640165383473316751405945979,
0.050983513068147724711653978602043935098,
0.051329988572197289553589172328429413028,
0.050983513068147724711653978602043935098,
0.049955904605640165383473316751405945979,
0.048282044852593269945728593484091106802,
0.046018195165831499160447037866106256843,
0.043239299548641835224671581272559706122,
0.040035431192303370939722384491687989794,
0.03650758740535271773541836637377855368 ,
0.032763072517720592813361690787132829428,
0.028910727678562592180000478947476949543,
0.02505626930118943357372884861433703918 ,
0.021297984345923545723699987775034969673,
0.01772300189523110663936122932682337705 ,
0.014404318807632635404680776503028027946,
0.011398705722202287851652080519215815002,
0.008745562014262435110434346086094592465,
0.006466728494857592983435790046087277005,
0.004567208755585410763500053832331104786,
0.003036697882958972090350346917375645717,
0.001851774076405747005216739786703783466,
0.000978577031845619463301666485222085612
};

class AltitudeFilter
{
public:
    AltitudeFilter() : AltitudeFilter( (const int)(sizeof(filter_coef_) / sizeof(filter_coef_[0])) , (const double *)filter_coef_) {};

    AltitudeFilter(const int filter_lenght, const double* filter_coef)
    {
        lenght = filter_lenght;

        data_counter = 0;
        data = new double[lenght];
        memset(data, 0, lenght * sizeof(data[0]));
        coef = new double[lenght];
        for (int i = 0; i < lenght; i++)
            coef[i] = filter_coef[i];

        coef_sum = 0;
        for (int i = 0; i < lenght; i++)
            coef_sum += coef[i];
    }
    AltitudeFilter(const int filter_lenght, const double filter_scale)
    {
        lenght = filter_lenght;

        data_counter = 0;
        data = new double[lenght];
        memset(data, 0, lenght * sizeof(data[0]));
        coef = new double[lenght];
        for (int i = 0; i < lenght; i++)
        {
            // Hamming window
            double w = 0.54 - 0.46 *cos(6.2832 * i / (lenght - 1)); 
            // sinc function
            double x = i - ((double)filter_lenght) / 2 + 0.5;
            double s;
            if (x == 0)
                s = 1;
            else
                s = ( sin(filter_scale * x) / x) / filter_scale;
            coef[i] = s * w;
        }
        coef_sum = 0;
        for (int i = 0; i < lenght; i++)
            coef_sum += coef[i];
    }

    ~AltitudeFilter() 
    {
        delete [] data;
        delete [] coef;
    }
    double  process(const double x)
    {
        double y = x;
        data_counter++;
        data[0] = x;
        double y1 = 0;
        for (int i = 0; i < lenght; i++)
            y1 += coef[i] * data[i];
        for (int i = lenght - 1; i >= 1; i--)
            data[i] = data[i - 1];
        y1 /= coef_sum;
        //if (data_counter >= lenght)
            y = y1;

        return y;
    }
private:

    int    lenght;
    double *data;
    double *coef;
    double coef_sum;
    int    data_counter;

};

#endif
