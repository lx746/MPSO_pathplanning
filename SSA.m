function [gBestScore, gBest, cg_curve] = SSA(N, iter, lb, ub, dim, fobj, G)
% SSA - Sparrow Search Algorithm
% N: population size
% iter: maximum iterations
% lb, ub: lower/upper bounds (scalar or vector)
% dim: dimension
% fobj: objective function handle
% G: (unused here, for compatibility)

    P_percent = 0.2;    % Producer proportion

    % Ensure bounds are vectors
    lb = lb .* ones(1, dim);
    ub = ub .* ones(1, dim);

    % Initialization
    x = zeros(N, dim);
    fit = zeros(N, 1);
    for i = 1 : N
        x(i, :) = lb + (ub - lb) .* rand(1, dim);
        fit(i) = fobj(x(i, :));
    end
    pFit = fit;
    pX = x;
    [gBestScore, bestI] = min(fit);
    gBest = x(bestI, :);

    cg_curve = zeros(1, iter);

    pNum = round(N * P_percent);    % Producer count

    for t = 1 : iter
        [~, sortIndex] = sort(pFit);
        [fmax, B] = max(pFit);
        worse = x(B, :);

        r2 = rand();
        if r2 < 0.8
            for i = 1 : pNum  % Producer update
                r1 = rand();
                x(sortIndex(i), :) = pX(sortIndex(i), :) * exp(-(i) / (r1 * iter));
                x(sortIndex(i), :) = Bounds(x(sortIndex(i), :), lb, ub);
                fit(sortIndex(i)) = fobj(x(sortIndex(i), :));
            end
        else
            for i = 1 : pNum
                x(sortIndex(i), :) = pX(sortIndex(i), :) + randn(1) * ones(1, dim);
                x(sortIndex(i), :) = Bounds(x(sortIndex(i), :), lb, ub);
                fit(sortIndex(i)) = fobj(x(sortIndex(i), :));
            end
        end

        [fMMin, bestII] = min(fit);
        bestXX = x(bestII, :);

        for i = (pNum + 1) : N
            A = floor(rand(1, dim) * 2) * 2 - 1;
            if i > (N / 2)
                x(sortIndex(i), :) = randn(1) * exp((worse - pX(sortIndex(i), :)) / (i^2));
            else
                % Avoid singular matrix
                Aprod = A' * (A * A')^(-1);
                if any(isnan(Aprod(:))) || any(isinf(Aprod(:)))
                    Aprod = eye(dim);
                end
                x(sortIndex(i), :) = bestXX + abs(pX(sortIndex(i), :) - bestXX) .* (Aprod' * ones(1, dim));
            end
            x(sortIndex(i), :) = Bounds(x(sortIndex(i), :), lb, ub);
            fit(sortIndex(i)) = fobj(x(sortIndex(i), :));
        end

        idx = randperm(numel(sortIndex));
        b = sortIndex(idx(1:min(20, N)));
        for j = 1 : length(b)
            if pFit(b(j)) > gBestScore
                x(b(j), :) = gBest + randn(1, dim) .* abs(pX(b(j), :) - gBest);
            else
                x(b(j), :) = pX(b(j), :) + (2 * rand(1) - 1) * abs(pX(b(j), :) - worse) / (pFit(b(j)) - fmax + 1e-50);
            end
            x(b(j), :) = Bounds(x(b(j), :), lb, ub);
            fit(b(j)) = fobj(x(b(j), :));
        end

        for i = 1 : N
            if fit(i) < pFit(i)
                pFit(i) = fit(i);
                pX(i, :) = x(i, :);
            end
            if pFit(i) < gBestScore
                gBestScore = pFit(i);
                gBest = pX(i, :);
            end
        end
        cg_curve(t) = gBestScore;
    end
end

function s = Bounds(s, Lb, Ub)
    % Apply bounds elementwise
    s = min(max(s, Lb), Ub);
end