[uniqueTime, ind] = unique(time, 'stable');

timelsim = uniqueTime(1):0.01:uniqueTime(end);
FLRoadInterp = interp1(uniqueTime, FLRoad(ind), timelsim)';
FRRoadInterp = interp1(uniqueTime, FRRoad(ind), timelsim)';
RLRoadInterp = interp1(uniqueTime, RLRoad(ind), timelsim)';
RRRoadInterp = interp1(uniqueTime, RRRoad(ind), timelsim)';
timelsim = timelsim';