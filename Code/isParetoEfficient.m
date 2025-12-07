function isEff = isParetoEfficient(costs)
% Each row in 'costs' is a point [D, F, V]
% Returns a logical vector indicating Pareto-efficient points

nPoints = size(costs,1);
isEff = true(nPoints,1);

for i = 1:nPoints
    if isEff(i)
        % Any point that dominates i?
        isDominated = all(bsxfun(@le, costs, costs(i,:)), 2) & ...
                      any(bsxfun(@lt, costs, costs(i,:)), 2);
        isEff(isDominated) = false;
    end
end
end
