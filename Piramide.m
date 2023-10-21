classdef Piramide
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Points
        Faces
    end
    
    methods
        function obj = Piramide(type)

            if type == 1
                obj.Points = [  0	-0.500000000000000	0.500000000000000	0	0
                                0	-1	-1	-1	-1
                                0	0	0	-0.500000000000000	0.500000000000000
                                1	1	1	1	1
                             ];
            else 
                if type == 2
                    obj.Points = [  0	1.11602540378444	0.616025403784439	0.866025403784439	0.866025403784439
                                    0	0.0669872981077804	0.933012701892219	0.500000000000000	0.500000000000000
                                    0	0	0	-0.500000000000000	0.500000000000000
                                    1	1	1	1	1
                                 ];
                else 
                    obj.Points = [  0	-0.616025403784438	-1.11602540378444	-0.866025403784439	-0.866025403784439
                                    0	0.933012701892220	0.0669872981077812	0.500000000000000	0.500000000000000
                                    0	0	0	-0.500000000000000	0.500000000000000
                                    1	1	1	1	1
                                 ];
                end
            end

            obj.Faces = [2 4 1 1
                         3 4 1 1
                         3 5 1 1
                         2 5 1 1
                         2 4 3 5];
        end
        function [P, F] = getPointsFaces(obj)
            P = obj.Points;
            F = obj.Faces;
        end
    end
end

