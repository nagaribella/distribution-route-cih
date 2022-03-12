function [Route, QRoute, DRoute] = CIH_Sweep(Data_Coor, Data_Dist, Data_Demand, Capacity, Direct)
% Cheapest Insertion Heuristic with Sweep Clustering
% Capacitated Vehicle Routing Problem
% 1. Data_Coor is data Latidute Longitude Coordinates of every node (Depot and
% Customers). This input must be a matrix with 2 columns (latitude and
% longitude).
% 2. Data_Dist is data distance between 2 nodes. This input must be a matrix
% (N+1) x (N+1), where N is the number of customers.
% 3. Data_Demand is number of customer demand. This input must be a column
% vector. Demand of Depot must be included which value is zero (0).
% 4. Capacity is the capacity of the vehicle.
% 5. Direct is a direction of sweeping clustering. (1) Forward/
% counterclockwise, (0) Backward/clockwise
if nargin == 4 %CIH_Sweep(Data_Coor, Data_Dist, Data_Demand, Capacity)
    Direct = 1;
end
N = size(Data_Dist,1)-1; %Number of customers
%Calculate theta
for i = 1:N
    x = Data_Coor(i+1,2) - Data_Coor(1,2);
    y = Data_Coor(i+1,1) - Data_Coor(1,1);
    if x > 0 && y > 0 %Quadrant 1
        Theta(i) = atand(y/x);
    elseif x < 0 && y > 0 %Quadrant 2
        Theta(i) = 180 + atand(y/x);
    elseif x < 0 && y < 0 %Quadrant 3
        Theta(i) = 180 + atand(y/x);
    elseif x > 0 && y < 0 %Quadrant 4
        Theta(i) = 360 + atand(y/x);
    elseif y == 0 %x axis
        if x >= 0
            Theta(i) = 0;
        else
            Theta(i) = 180;
        end
    elseif x == 0 %y axis
        if y > 0
            Theta(i) = 90;
        else
            Theta(i) = 270;
        end
    end
end
%Sorting theta & clustering
if Direct == 1
    [rTheta, idx] = sort(Theta,'ascend');
elseif Direct == 0
    [rTheta, idx] = sort(Theta,'descend');
end
k = 1; a = 1; all = 0;
for i = 1:N
    SumQ = sum(Data_Demand(idx(a:i)+1));
    if SumQ > Capacity
        Cluster{k} = idx(a:i-1);
        QRoute(k) = sum(Data_Demand(idx(a:i-1)+1));
        k = k+1;
        a = i;
        all = all + length(Cluster{k-1});
    end
end
if all < N
    Cluster{k} = idx(a:N);
    QRoute(k) = sum(Data_Demand(idx(a:N)+1));
end
K=size(Cluster,2); %Number of vehicles
for k = 1:K
    Node = Cluster{k};
    %Initial sub tour
    Cijk = []; Sub_Tour = [];
    for i = 1:length(Node)
        Cijk(i) = Data_Dist(1,Node(i)+1) + Data_Dist(Node(i)+1,1);
    end
    id = find(Cijk==min(Cijk));
    Sub_Tour = [0 Node(id(1)) 0];
    Cij = min(Cijk);
    Node = setxor(Node, Node(id(1)));
    while ~isempty(Node)
        %Insertion
        a = 1; Temp = []; Cijk = [];
        for i = 2:length(Sub_Tour)
            for j = 1:length(Node)
                Temp(a,:) = [Sub_Tour(1:i-1) Node(j) Sub_Tour(i:end)];
                Cijk(a) = Cij - Data_Dist(Sub_Tour(i-1)+1,Sub_Tour(i)+1) + ...
                    Data_Dist(Sub_Tour(i-1)+1,Node(j)+1) + Data_Dist(Node(j)+1,Sub_Tour(i)+1);
                a = a+1;
            end
         end
        %Find minimum Cij
        id = find(Cijk==min(Cijk));
        NP = setxor(Sub_Tour, Temp(id(1),:));
        Sub_Tour = Temp(id(1),:);
        Cij = min(Cijk);
        Node = setxor(Node, NP);
    end
        %Route
    Route{k} = Sub_Tour;
    DRoute(k) = Cij;
end 