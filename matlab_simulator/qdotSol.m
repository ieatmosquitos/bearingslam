% ***********************************************************
% *                                                         *
% * Gabriele Gualandi & Filippo Bianchi                     *
% * djgwala@yahoo.it                                        *
% *                                                         *
% *         PROJECT IN:                  WITH PROFESSOR:    *
% * Autonomous & Mobile Robotics        Giuseppe Oriolo     *
% * Robotics 2                          Alessandro De Luca  *
% *                                                         *
% * Master in Artificial Intelligence and Robotics,         *
% * Sapienza Università di Roma, 2011                       *
% *                                                         *
% * launch "main.m" first                                   *
% ***********************************************************

function [qdot] = qdotSol(M,stateSize,outVarMaxAbsValue)

% Dati una serie di constraint ODE e ODI, ordinati in una cell M in base 
% alla loro priorità, risolve il problema QP in base ai vincoli definiti in 
% M e ritorna la soluzione x (nel nostro caso qdot) del problema di
% minimizzazione risolto dal QP.
% La cell è strutturata in questo modo:
%
% [A1] .. [Ci] ..
% [b1] .. [di] ..
% [1]  .. [k]  ..
% ['e'].. ['i']..
%
% dove la terza riga rappresenta la priorotà associata al vincolo mentre la
% quarta contiene un flag che determina se si tratta di un ODI o un ODE
%--------------------------------------------------------------------------


% init A_k-1 b_k-1 C_k-1 d_k-1
Ak1=[];
bk1=[];
Ck1=[];
dk1=[];

useAMPL=1; % If true, the minimization will be solved with external solver AMPL,
% instead of quadprog.

global mDBG; %DEBUG
global mDBG1; %DEBUG

taskPr=cell2mat(M(3,:));
taskType=cell2mat(M(4,:));

options=optimset('LargeScale','off');


for p=1:max(taskPr)
    Ak_originale=0;

    %trovo ODE con priorità p
    t1 = find(taskPr == p); % Indici colonne con priorità p
    if isempty(t1)
        msgbox(sprintf('Cannot found any task with priority %i, so probably you will obtain some error! Change your task priority so that you have not empty levels.',p), 'WARNING');
    end
    t2 = find(taskType == 'e'); % indici colonne task = p che di uguaglianza
    t3 = intersect(t1,t2);
    
    Ak = cell2mat(M(1,t3)'); 
    if isempty(Ak)
        Ak_originale=Ak;
        Ak = zeros(1,stateSize);
    end
        
    bk = cell2mat(M(2,t3)');
    if isempty(bk)
        bk = zeros(1,1);
    end
    
    %trovo ODI con priorità p
    t2 = find(taskType== 'i');
    t3 = intersect(t1,t2);
    Ck = cell2mat(M(1,t3)');

    dk = cell2mat(M(2,t3)');     

    if isempty(Ck1)
        A = [Ck, -eye(size(Ck,1))];
    else
        A = [Ck, -eye(size(Ck,1));
            Ck1, zeros(size(Ck1,1),size(Ck,1))];
    end
    b = [dk;
        dk1];
    Aeq = [Ak1, zeros(size(Ak1,1),size(Ck,1))];
    beq = bk1;

    
    
if useAMPL    
    
    nSlackV = size(Ck,1);
    
    dimAugmentedState = stateSize + nSlackV;
    xi = [];
    for i=1:dimAugmentedState
        t = sym(['x' int2str(i)]);
        xi = [xi; t];
    end
    
    if nSlackV > 0
        slackV = xi(7:end);
    end
    
    
    if ~isempty(find(Ak,1)) 
        Axb = vpa(Ak * xi(1:stateSize) - bk);
        if nSlackV == 0
            funcMin = (1/2)*(Axb .'*Axb );
        else
            funcMin = (1/2)*(Axb .'*Axb ) + (1/2)*(slackV.'*slackV);
        end        
    else
        funcMin = (1/2)*(slackV.'*slackV);
    end
    
    fileOut = 'amplProblem.mod';
    fileIn = 'amplResults';
    
    createFileAMPL(fileOut,xi,funcMin,A,b,Aeq,beq,outVarMaxAbsValue);
    
    if ispc
        dos 'ampl.exe amplProblemScript.go';
    elseif ismac
        dos 'ampl amplProblemScript.go';
    else 
        disp('STOP!! Sorry, only Windows and Mac OS are supported... Please break.');
        pause;
    end
   
   
    % Prelevo i risultati dal file
    fid=fopen(fileIn);
    qdot=fscanf(fid,'x1 = %f x2 = %f x3 = %f x4 = %f x5 = %f x6 = %f');
    fclose(fid);

else
        %definisco matrici da passare al QP solver
    H = blkdiag((Ak'*Ak), eye(size(Ck,1)));
    f = [-Ak' * bk;
        zeros(size(Ck,1),1)];


    lb=[];
    ub=[];
    
    if isempty(qdotold)
        x = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
    else
        x0=[qdotold;zeros(size(Ck,1),1)];
        x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    end
    qdot = x(1:stateSize);
    
    if ~isempty(qdotold)
        qdotold=qdot;
    end
    
    
    
end


    
    %aggiorno A_k-1 b_k-1 C_k-1 d_k-1
    if isempty(Ak_originale)
        if isempty(Ck)
            disp(['task di priorità ', num2str(p),' non assegnato']);
        else
            Ak1 = [Ak1;Ck(Ck*qdot>dk,:)];
            bk1 = [bk1;Ck(Ck*qdot>dk,:)*qdot];
            Ck1 = [Ck1;Ck(Ck*qdot<=dk,:)];
            dk1 = [dk1;dk(Ck*qdot<=dk)];
        end
    else
        if isempty(Ck)
            Ak1 = [Ak1;Ak];
            bk1 = [bk1;Ak*qdot];
        else

            Ak1 = [Ak1;Ak;Ck(Ck*qdot>dk,:)];
            bk1 = [bk1;Ak*qdot;Ck(Ck*qdot>dk,:)*qdot];
            Ck1 = [Ck1;Ck(Ck*qdot<=dk,:)];
            dk1 = [dk1;dk(Ck*qdot<=dk)];
        end
    end
    
%     % DEBUG
%     if ~isempty(Ck) && ~isempty(Aeq)
%         newCell1 = A(:,1:stateSize) * qdot - b;
%         newCell2 = Aeq(:,1:stateSize)*qdot-beq;
%         mDBG=cat(1,mDBG,[newCell1,newCell2',p]);
%     else
%         if ~isempty(Ck)
%             newCell1=A(:,1:stateSize) * qdot - b;
%             mDBG=cat(1,mDBG,newCell1);
%         end
%     end
%     if ~isempty(Ak_originale)
%         newCell=Ak*qdot-bk;
%         mDBG1=cat(1,mDBG1,[newCell',p]);
%     end 
    
end

   
    
end


