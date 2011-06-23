// Heap.h: interface for the CHeap class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_HEAP_H__D95FA3CF_0DC1_4A82_9DE2_B872CF7EEAFA__INCLUDED_)
#define AFX_HEAP_H__D95FA3CF_0DC1_4A82_9DE2_B872CF7EEAFA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

const int MAX_NODE	= 65535;

typedef struct node       Node;
struct node{
 int    type;
 int    type2;
 double pos[3];
 void * p;
 double v;
 
 Node * next;
 Node * prev;
};

class CHeap  // Min Heap 
{
public:
	int heapFind(int type);
    int heapFindPair(int type, int type2);
	bool heapEmpty();
	void heapCheck();
	Node* heapSelectMin();
	void heapConstruct(Node* a[], int N);
	void heapPrint();
	void heapDownheap(void* v);
	void heapUpheap(void* v);
	Node* heapNode(void* v);
	int heapIndex(void* v);
	void heapsort(Node* a[], int N);
	Node* replace(Node* v);
	Node* Remove();
	void downheap(int k);
	void insert(Node* v);
	void upheap(int k);
	void construct(Node* b[], int M);
	int NodeCompare(Node* a, Node* b);
	int N;
    int capacity;
	Node** a;
	CHeap();
	virtual ~CHeap();

};

#endif // !defined(AFX_HEAP_H__D95FA3CF_0DC1_4A82_9DE2_B872CF7EEAFA__INCLUDED_)
