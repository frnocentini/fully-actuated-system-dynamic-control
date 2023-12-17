classdef PANDA_sym < SerialLink
 
	properties
	end
 
	methods
		function ro = PANDA_sym()
			objdir = which('PANDA_sym');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@PANDA_sym','matPANDA_sym.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
