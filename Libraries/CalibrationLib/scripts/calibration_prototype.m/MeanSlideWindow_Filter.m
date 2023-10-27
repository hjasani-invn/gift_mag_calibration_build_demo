function flt_vec = MeanSlideWindow_Filter (vec, span)
    n = fix(span / 2);
    sz = length(vec);
    for i = sz:-1:1
        ni = min(min(i-1, n), sz-i);
        flt_vec(i) = mean(vec(i-ni:i+ni));
    end
end