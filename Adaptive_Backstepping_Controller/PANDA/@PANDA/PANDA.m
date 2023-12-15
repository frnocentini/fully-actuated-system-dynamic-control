classdef PANDA < SerialLink
 
	properties
	end
 
	methods
		function ro = PANDA()
			objdir = which('PANDA');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@PANDA','matPANDA.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
