[projectRootDir,~,~] = fileparts(mfilename('fullpath'));
idcs = strfind(projectRootDir,'\'); % find \ string
userpath = projectRootDir(1:idcs(end-2)-1); % get parent directory