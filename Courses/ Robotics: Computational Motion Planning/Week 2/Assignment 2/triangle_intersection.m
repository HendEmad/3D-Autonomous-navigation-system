function flag = triangle_intersection(P1, P2)
    % triangle_test : returns true if the triangles overlap and false otherwise
    
    % Define the SideOfLine function
    SideOfLine = @(p1, p2, p3) sign((p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1)));
    
    % DrawTriangle(P1, P2);
    % pause;
    
    flag = true;
    for k = 1:2 % iterate over P1 and P2
        n = nchoosek(1:size(P1, 1), 2); % go through all the pairs of vertices of a triangle that form a line
        
        for i = 1:size(n,1)
            % check the side of the 3rd vertex of P1 to the line formed by
            % the 1st and 2nd vertex.
            sideA = SideOfLine(P1(n(i,1),:), P1(n(i,2), :), P1(setdiff(1:size(P1,1), n(i,:)),:));
            
            % check all the vertices of P2 against the line formed by each
            % pair of the two vertices of P1
            sideB = zeros(size(P2,1),1);
            for j = 1:size(P2, 1)
                sideB(j) = SideOfLine(P1(n(i,1),:), P1(n(i,2),:), P2(j,:));
            end
            
            % check whether all the vertices of P2 are on the opposite side
            % of the 3rd vertex of P1. If yes, return flag = "no overlap".
            % First, these vertices must be on the same side, and 2nd, they
            % must be on the opposite side.
            if length(unique(sideB)) == 1 && sum(sideB ~= sideA) == size(P2, 1)
                flag = false;
                return;
            end
        end
        
        % do the above again by swapping P1 and P2
        [P1, P2] = deal(P2, P1);
    end
end