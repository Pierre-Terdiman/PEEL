///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICESELECTIONDIALOG_H
#define ICESELECTIONDIALOG_H

	enum SelectionButtonType
	{
		SBT_UNDEFINED,
		SBT_OK,
		SBT_CANCEL,
		SBT_SELECT_ALL,
		SBT_SELECT_NONE,
		SBT_SELECT_INVERT,
		SBT_FILTER_ALL,
		SBT_FILTER_NONE,
		SBT_FILTER_INVERT,
	};

	class ICEGUI_API SelectionDialog;
	class ICEGUI_API SelectionButton : public IceButton
	{
		public:
							SelectionButton(const ButtonDesc& desc);
		virtual				~SelectionButton();

		virtual	void		OnClick();

		SelectionDialog*	mDlg;
	};

	class ICEGUI_API SelectionCheckBox : public IceCheckBox
	{
		public:
							SelectionCheckBox(const CheckBoxDesc& desc);
		virtual				~SelectionCheckBox();

		virtual	void		OnClick();

		SelectionDialog*	mDlg;
	};

	class ICEGUI_API SelectionWindow : public IceListBox
	{
		public:
							SelectionWindow(const ListBoxDesc& desc);
		virtual				~SelectionWindow();

		virtual	void		OnListboxEvent(ListBoxEvent event);

		SelectionDialog*	mMain;
	};

	struct ICEGUI_API SelectionDialogDesc : WindowDesc
	{
		SelectionDialogDesc() : mFilters(null), mMultiSelection(true)	{}

		const char*				mFilters;
		bool					mMultiSelection;
	};

#define MAX_NB_FILTERS	10
	class ICEGUI_API SelectionDialog : public IceWindow
	{
		public:
							SelectionDialog(const SelectionDialogDesc& desc);
		virtual				~SelectionDialog();

		virtual		void	OnSelected(void* user_data)
							{
							}

					udword	Add(const char* name, udword group, void* user_data, BOOL selected);
					udword	Update();
					void	Resize();

//		private:
		SelectionWindow*	mSelectionWindow;
		SelectionButton*	mSelectButton;
		SelectionButton*	mCancelButton;
		SelectionButton*	mSelectAllButton;
		SelectionButton*	mSelectNoneButton;
		SelectionButton*	mSelectInvertButton;
		SelectionButton*	mFilterAllButton;
		SelectionButton*	mFilterNoneButton;
		SelectionButton*	mFilterInvertButton;
		udword				mNbFilters;
		SelectionCheckBox*	mFilters[MAX_NB_FILTERS];
		SelectionCheckBox*	mSortByName;
		IceGroupBox*		mGroupBox;
		Container			mItems;
		udword				mMaxLength;
		bool				mInitialUpdate;
	};

#endif	// ICESELECTIONDIALOG_H