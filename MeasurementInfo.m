classdef MeasurementInfo
    properties
        landmark_id
        type
    end
    
    methods
        function obj = MeasurementInfo(landmark_id, type)
            obj.landmark_id = landmark_id;
            obj.type = type;
        end
    end
end

