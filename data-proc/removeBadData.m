%% remove bad data in a row (containing -1)
function data = removeBadData(data_raw)

data = data_raw;
bad_rows = [];
for row = 1:size(data,1)
    if ismember(-1,data(row,end-2:end))
        bad_rows = [bad_rows, row];
    end
end
data(bad_rows,:) = [];