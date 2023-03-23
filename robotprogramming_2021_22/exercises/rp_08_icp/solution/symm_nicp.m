1;

function T=v2t(v)
  s=sin(v(3));
  c=cos(v(3));
  T=[c, -s, v(1);
     s,  c, v(2);
     0,  0, 1];
endfunction

function v = transform(X, p)
  v=p;
  v(1:2)=X(1:2,1:2)*p(1:2)+X(1:2,3);
  v(3:4)=X(1:2,1:2)*p(3:4);
endfunction;

function V = transformAll(X, P)
  V=P;
  for (i=1:size(P,2))
    V(:,i)=transform(X,P(:,i));
  endfor;
endfunction;

function s=skew(v)
  s=[v(2); -v(1)];
endfunction;

function [e, J] = errorAndJacobian(m, f)
  e=zeros(3,1);
  pm=m(1:2);
  pf=f(1:2);
  nm=m(3:4);
  nf=f(3:4);
  e(1)=(pm-pf)' * (nm+nf);
  e(2:3)=nm-nf;
  J=zeros(3,3);
  J(1,1:2)=(nm+nf)';
  J(1,3)=pm'*skew(nf)+pf'*skew(nm);
  #J(2:3,3)=-skew(nm);
endfunction;

function [X, chi]=oneRound(X,M,F)
  H=zeros(3,3);
  b=zeros(3,1);
  chi=0;
  for (i=1:size(M,2))
    m=transform(X,M(:,i));
    f=F(:,i);
    [e,J]=errorAndJacobian(m,f);
    H+=J'*J;
    b+=J'*e;
    chi+=e'*e;
  endfor;
  dx=-H\b;
  X=v2t(dx)*X;
endfunction

function P=makePoints(np)
  P=(rand(4, np)-0.5)*100;
  N=P(3:4,:);
  S=sqrt(sum(N.*N));
  P(3:4,:)./=S;
endfunction

function [X_est, chis] = testICP(np, X, rounds)
  F=makePoints(np);
  M=transformAll(X,F);
  X_est=eye(3);
  chis=zeros(1,rounds);
  for (i=1:rounds)
    [X_est, chi]=oneRound(X_est, M, F);
    chis(i)=chi;
  endfor
endfunction;
