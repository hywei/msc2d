// Heap.cpp: implementation of the CHeap class.
//
//////////////////////////////////////////////////////////////////////

#include "heap.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHeap::CHeap()
{
	a = new Node*[MAX_NODE];
	N = 0;
    capacity = MAX_NODE;
}

CHeap::~CHeap()
{
	if(a)
		delete a;
}

int CHeap::NodeCompare(Node *a, Node *b)
{
	if( a->v < b->v )
		return -1;
	if( a->v > b->v )
		return  1;
	return 0;
}

void CHeap::construct(Node *b[], int M)
{
	for (N=1;N<=M;N++) 
		a[N]=b[N];
	if( M== 0 )
		N = 0;
}

void CHeap::upheap(int k)
{  
	Node* v;
	v = a[k];
	a[0] = NULL;
	while( k > 1 && NodeCompare(a[k/2],v) >= 0)
    {
		a[k] = a[k/2];k=k/2;
    }
    a[k]=v;
}

void CHeap::insert(Node *v)
{
    if(N == capacity-1)   // Not enough space for nodes
    {
        Node** b = new Node*[capacity+MAX_NODE];
        assert(b != NULL);
        for(int i = 0; i < capacity; ++ i)
            b[i] = a[i];
        delete[] a;
        a = b;
        capacity += MAX_NODE;
    }
    a[++N]=v;
	upheap(N);
}

void CHeap::downheap(int k)
{
	int j;
	Node * v;
	v = a[k];
	while( k <= N/2 ){
		j= k+k;
		if(j<N && NodeCompare(a[j],a[j+1])>0) j++;
		if(NodeCompare(v,a[j])<=0) break;
		a[k]= a[j];k=j;
	}
	
	a[k] = v;
}

Node* CHeap::Remove()
{  
	Node * v = a[1];
	a[1] =  a[N--];
	downheap(1); 
	return v;
}

Node* CHeap::replace(Node *v)
{
	a[0] = v;
	downheap(0);
	return a[0];
}
// 将数据a根据a.v进行堆排序，a中将包含排序结果
void CHeap::heapsort(Node *a[], int N)
{
	int k;
	construct(a,0);
	for( k= 1;k<=N;k++) insert( a[k] );
	for(k=1; k<=N; k++) a[k] = Remove();
}

int CHeap::heapIndex(void *v)
{
	int i;
	for( i = 1; i <= N; i ++ ) if( a[i]->p == v ) return i;
	return 0;
}

Node* CHeap::heapNode(void *v)
{
	int i;
	for( i = 1; i <= N; i ++ ) if( a[i]->p == v ) return a[i];
	return NULL;
}

void CHeap::heapUpheap(void *v)
{
	int i = heapIndex( v );
	if( !i ){
		fprintf(stderr, "There is no such element in heap \n");
		return;
	}
	upheap(i);
}

void CHeap::heapDownheap(void *v)
{
	int i = heapIndex( v );
	if( !i ){
		fprintf(stderr, "There is no such element in heap \n");
		return;
	}
	downheap(i);
}

void CHeap::heapPrint()
{
	int i;
	for( i = 1; i <=N; i++ )		
		printf("root %f left-child %f right-child %f\n", a[i]->v,(i*2>N)?-1:a[i*2]->v, (i*2+1>N)?-1:a[i*2+1]->v);
}

void CHeap::heapConstruct(Node *a[], int N)
{
	int k;
	construct(a,0);
	for( k= 1;k<=N;k++) insert( a[k] );
}
/* if new value > old value  downheap , if new value < old value, up heap */
Node* CHeap::heapSelectMin()
{
	return Remove();
}

void CHeap::heapCheck()
{
	int k,flag;
	
	flag = 1;
	for( k= 1;k<=N;k++){
		if( 2*k   <=N) flag &= (a[k]->v <= a[2*k  ]->v);
		if( 2*k+1 <=N) flag &= (a[k]->v <= a[2*k+1]->v);
		if( !flag ){
			printf("Error in heapCheck");
			return;
		} 
	}
	printf("good in heapCheck" );
}

bool CHeap::heapEmpty()
{
	return (N<1) ;
}

int CHeap::heapFind(int type)
{
	int i;
	for( i = 1; i <= N; i ++ ) if( a[i]->type == type ) return i;
	return 0;
}

int CHeap::heapFindPair(int type, int type2)
{
    for(int i = 1; i <= N; ++ i)
    {
        if(a[i]->type == type && a[i]->type2 == type2)
            return i;
    }
    return 0;
}
